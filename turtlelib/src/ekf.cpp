#include <iostream>
#include <armadillo>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"

namespace turtlelib
{
    // State variable includes configuration of the robot, and X and y positions of the landmarks
    EKFSlam::EKFSlam(): 
    q{num_dof, arma::fill::zeros}, m{2 * num_landmarks, arma::fill::zeros}, Xi{num_dof + 2 * num_landmarks, arma::fill::zeros}, sigma{num_dof + 2 * num_landmarks, num_dof + 2 * num_landmarks, arma::fill::zeros}
    {
        initialize_covariance();
    }

    EKFSlam::EKFSlam(Pose2D turtle_pose_0): 
    q{num_dof, arma::fill::zeros}, m{2 * num_landmarks, arma::fill::zeros}, Xi{num_dof + 2 * num_landmarks, arma::fill::zeros}, sigma{num_dof + 2 * num_landmarks, num_dof + 2 * num_landmarks, arma::fill::zeros}
    {
        initialize_covariance();
        initialize_pose(turtle_pose_0);
    }

    void EKFSlam::initialize_covariance()
    {
        arma::mat sigma_0_q = arma::zeros<arma::mat>(num_dof, num_dof); // We are absolutely sure about the initial pose
        arma::mat sigma_0_m = arma::eye(2 * num_landmarks, 2 * num_landmarks) * 1e6; // Uncertainity in sensing, very high with no knowledge of obstacles
        arma::mat zeros_12 = arma::zeros<arma::mat>(num_dof, 2*num_landmarks); // Zeros due to sensing and localization noise being independent
        arma::mat zeros_21 = arma::zeros<arma::mat>(2*num_landmarks, num_dof); // Zeros due to sensing and localization noise being independent
        sigma =
            arma::join_vert(
            arma::join_horiz(sigma_0_q, zeros_12), 
            arma::join_horiz(zeros_21, sigma_0_m));
    }

    void EKFSlam::initialize_pose(Pose2D turtle_pose_0)
    {
        q(0) = turtle_pose_0.theta;
        q(1) = turtle_pose_0.x;
        q(2) = turtle_pose_0.y;
        
        update_state_vector();
    }

    void EKFSlam::update_state_vector()
    {
        // Populate with pose vector
        for (int pose_index = 0; pose_index < num_dof; pose_index++)
        {
            Xi(pose_index) = q(pose_index);
        }
        // Populate with map vector
        for (int landmark_index = 0; landmark_index < num_landmarks; landmark_index++)
        {
            Xi(num_dof + 2*landmark_index) = m(2*landmark_index); // X coordinate of landmark
            Xi(num_dof + 2*landmark_index + 1) = m(2*landmark_index + 1); // Y coordinate of landmark
        }
    }

    void EKFSlam::update_pose_and_map()
    {
        // Populate pose vector
        for (int pose_index = 0; pose_index < num_dof; pose_index++)
        {
            q(pose_index) = Xi(pose_index);
        }
        // Populate map vector
        for (int landmark_index = 0; landmark_index < num_landmarks; landmark_index++)
        {
            m(2*landmark_index) = Xi(num_dof + 2*landmark_index); // X coordinate of landmark
            m(2*landmark_index + 1) = Xi(num_dof + 2*landmark_index + 1); // Y coordinate of landmark
        }
    }

    /// \brief - Predict the state and covariance, every time a twist input is given
    void EKFSlam::predict(Twist2D twist)
    {
        // Check for proper twist
        if(!almost_equal(twist.y , 0.0))
        {
            throw std::runtime_error("Improper twist for estimation!");
        }

        // Update twist
        u(0) = twist.omega;
        u(1) = twist.x;
        u(2) = 0.0;

        // First we predict the covariance ˆΣ-_t using current A_t which is calculated using the previous state ξ_{t−1}, and current input u_t.
        // ˆΣ¯_t = A_t ˆΣ_{t−1} A_t^{T} + Q-,

        // Calculate A matrix.
        arma::mat zeros_12{num_dof, 2 * num_landmarks, arma::fill::zeros};
        arma::mat zeros_21{2 * num_landmarks, num_dof, arma::fill::zeros};
        arma::mat zeros_22{2 * num_landmarks, 2 * num_landmarks, arma::fill::zeros};
        arma::mat pose_state_matrix(num_dof, num_dof, arma::fill::zeros);

        // Zero rotational velocity
        if (almost_equal(u(0), 0.0)) 
        {     
            pose_state_matrix(1, 0) = -u(1) * sin(q(0));
            pose_state_matrix(2, 0) = u(1) * cos(q(0));
        } 
        // Non-zero rotational velocity
        else 
        {   
            pose_state_matrix(1, 0) = -(u(1) / u(0)) * cos(q(0)) + (u(1) / u(0)) * cos(normalize_angle(q(0) + u(0)));
            pose_state_matrix(2, 0) = -(u(1) / u(0)) * sin(q(0)) + (u(1) / u(0)) * sin(normalize_angle(q(0) + u(0)));
        }

        A = I +
            arma::join_vert(
            arma::join_horiz(pose_state_matrix, zeros_12),
            arma::join_horiz(zeros_21, zeros_22));

        // Represent process noise Q as Q_bar
        arma::mat Q_bar =
            arma::join_vert(
            arma::join_horiz(Q, zeros_12), 
            arma::join_horiz(zeros_21, zeros_22));
        
        // Update covariance matrix
        sigma = A * sigma * A.t() + Q_bar;
        
        // Now we predict the mean pose ˆξ-_t using the current pose ξ_{t−1}
        // ˆξ¯_t = g(ˆξ_{t−1}, u_t, 0)
        
        // Predict mean pose nonlinearly
        Transform2D pose_as_tf_Twb{Vector2D{q(1), q(2)}, q(0)}; // Original mean pose
        Transform2D twist_as_tf_TbB = integrate_twist(twist);   // Change in mean pose
        Transform2D newpose_as_tf_TwB = pose_as_tf_Twb * twist_as_tf_TbB;  // Final mean pose

        // Update pose and state
        q(0) = newpose_as_tf_TwB.rotation();
        q(1) = newpose_as_tf_TwB.translation().x;
        q(2) = newpose_as_tf_TwB.translation().y;
        update_state_vector();

        // // Check covariance matrix (symmetric and positive semi-definite)
        // // Check if symmetric
        // if (!(sigma.is_symmetric(1e-8)))
        // {
        //     throw std::runtime_error("Covariance is Assymetric!!!");
        // }
        // // Check if positive semi-definite
        // arma::vec eigvals = arma::eig_sym(sigma);
        // if (!(arma::all(eigvals >= 0)))
        // {
        //     throw std::runtime_error("Covariance is not Positive Semi-Definite!!!");
        // }

    }

    /// \brief Correct the state and covariance using the Kalman gain, every time a landmark is sensed
    void EKFSlam::correct(double x, double y, size_t j)
    {
        // Convert relative measurements to range-bearing
        // r_j = (x^2 + y^2)^0.5
        // ϕ_j = atan2(y, x)
        double r_j = magnitude(Vector2D{x, y});
        double phi_j = std::atan2(y, x);      

        // If landmark has not been seen before, add it to the map
        // Since our pose variables are predictions (distributions), our map is also a prediction. 
        if (seen_landmarks.find(j) == seen_landmarks.end()) 
        {
            // Initialize the landmark prediction as x and y predictions in map frame
            m(2 * (j-1)) = q(1) + r_j * cos(phi_j + q(0)); // ˆm_{x,j}
            m(2 * (j-1) + 1) = q(2) + r_j * sin(phi_j + q(0)); // ˆm_{y,j}
            // Insert the new landmark index in the unordered_set
            seen_landmarks.insert(j);
            update_state_vector();
        }
        // Actual Measurement of that landmark. Not a distribution.
        z_i(0) = r_j;
        z_i(1) = phi_j;

        // Relative predictions of landmark position, as cartesian coordinates
        // δ_{x,j} = ˆm_{x,j} − ˆx_t
        // δ_{y,j} = ˆm_{y,j} − ˆy_t
        // d_j = δ_{x,j}^2 + δ_{y,j}^2
        Vector2D delta_j{m(2*(j-1)) - q(1), m(2*(j-1)+1) - q(2)};
        double d_j = std::pow(magnitude(delta_j), 2);

        // Relative predictions of landmark position, as range-bearing
        double r_j_hat = std::sqrt(d_j);
        double phi_j_hat = normalize_angle(atan2(delta_j.y, delta_j.x) - q(0));
        z_i_hat(0) = r_j_hat;
        z_i_hat(1) = phi_j_hat;

        // Calculate H matrix
        arma::mat small_H_first{2, num_dof, arma::fill::zeros}; // Dependence on pose
        arma::mat zeros_2_first{2, 2 * (j-1), arma::fill::zeros}; // Dependence on landmarks having smaller indices
        arma::mat small_H_second{2, 2, arma::fill::zeros}; // Dependence on sensed landmark
        arma::mat zeros_2_second{2, 2 * num_landmarks - 2 * j, arma::fill::zeros}; // Dependence on landmarks having larger indices

        small_H_first(0, 0) = 0.0;
        small_H_first(0, 1) = -delta_j.x / std::sqrt(d_j);
        small_H_first(0, 2) = -delta_j.y / std::sqrt(d_j);
        small_H_first(1, 0) = -1;
        small_H_first(1, 1) = delta_j.y / d_j;
        small_H_first(1, 2) = -delta_j.x / d_j;

        small_H_second(0, 0) = delta_j.x / std::sqrt(d_j);
        small_H_second(0, 1) = delta_j.y / std::sqrt(d_j);
        small_H_second(1, 0) = -delta_j.y / d_j;
        small_H_second(1, 1) = delta_j.x / d_j;

        H_i = arma::join_horiz(arma::join_horiz(small_H_first, zeros_2_first), arma::join_horiz(small_H_second, zeros_2_second));

        // Sensor noise matrix
        R = arma::mat{2, 2, arma::fill::eye} * R_noise;
        // Rj = R.submat(j, j, j + 1, j + 1);

        // Kalman gain
        // K_i = Σ¯_t H_{i}^{T} (H_i Σ¯_t H_{i}^{T} + R)^{-1}
        K_i = sigma * H_i.t() * (H_i * sigma * H_i.t() + R).i();

        // Update state to corrected prediction
        
        // Subtract z_i and z_i_hat correcctly
        arma::colvec z_i_diff{2, arma::fill::zeros};
        z_i_diff(0) = z_i(0) - z_i_hat(0);
        z_i_diff(1) = normalize_angle(z_i(1) - z_i_hat(1));

        // ξ_t = ˆξ¯_t + K_i (z^i_t − ˆz^i_t)
        Xi = Xi + K_i * (z_i_diff);    
        update_pose_and_map();

        // Update covariance
        // Σ_t = (I − K_i H_i) Σ¯_t
        sigma = (I - K_i * H_i) * sigma;

        // Srikanth is super cool
    }

    size_t EKFSlam::associate_index(Point2D measurement)
    {
        // For each measurement z_i
        // Convert relative measurements to range-bearing
        // r_i = (x^2 + y^2)^0.5
        // ϕ_i = atan2(y, x)
        double r_i = std::sqrt(measurement.x * measurement.x + measurement.y * measurement.y);
        double phi_i = std::atan2(measurement.y, measurement.x);      // Normalize ?? TODO ??

        // Create a temp map with new temp landmark
        arma::colvec m_temp{2*(num_landmarks+2), arma::fill::zeros};

        for(int index = 0; index < std::min(static_cast<int>(2*(N+1)), 2*num_landmarks); index++)
        {
            m_temp(index) = m(index);
        }
        // Initialize the landmark estimate x and y coordinates in the map
        m_temp(2*(N+1)) = q(1) + r_i * cos(phi_i + q(0));
        m_temp(2*(N+1) + 1) = q(2) + r_i * sin(phi_i + q(0));

        // m_temp(2*(N)) = q(1) + r_i * cos(phi_i + q(0));
        // m_temp(2*(N) + 1) = q(2) + r_i * sin(phi_i + q(0));

        // Actual measurements
        arma::colvec z_i_temp{2, arma::fill::zeros};
        z_i_temp(0) = r_i;
        z_i_temp(1) = phi_i;

        // Sensor noise matrix
        R = arma::mat{2, 2, arma::fill::eye} * R_noise; 

        std::vector<arma::mat> maha_distances{}; // Mahalanobis distance for each landmark
        std::vector<arma::mat> eu_distances{}; // euclidean distance for each landmark

        for (size_t k = 1; k <= static_cast<size_t>(std::min(static_cast<int>(N+1), num_landmarks)); k++)
        {            
            Vector2D delta_k{m_temp(2*(k-1)) - q(1), m_temp(2*(k-1)+1) - q(2)};
            double d_k = std::pow(magnitude(delta_k), 2);

            // Compute H_k 

            arma::mat H_k{2, num_dof+2*num_landmarks, arma::fill::zeros};

            arma::mat small_H_first{2, num_dof, arma::fill::zeros}; // Dependence on pose
            arma::mat zeros_2_first{2, 2 * (k-1), arma::fill::zeros}; // Dependence on landmarks having smaller indices
            arma::mat small_H_second{2, 2, arma::fill::zeros}; // Dependence on sensed landmark
            arma::mat zeros_2_second{2, 2 * num_landmarks - 2 * k, arma::fill::zeros}; // Dependence on landmarks having larger indices

            small_H_first(0, 0) = 0.0;
            small_H_first(0, 1) = -delta_k.x / std::sqrt(d_k);
            small_H_first(0, 2) = -delta_k.y / std::sqrt(d_k);
            small_H_first(1, 0) = -1;
            small_H_first(1, 1) = delta_k.y / d_k;
            small_H_first(1, 2) = -delta_k.x / d_k;

            small_H_second(0, 0) = delta_k.x / std::sqrt(d_k);
            small_H_second(0, 1) = delta_k.y / std::sqrt(d_k);
            small_H_second(1, 0) = -delta_k.y / d_k;
            small_H_second(1, 1) = delta_k.x / d_k;

            H_k = arma::join_horiz(arma::join_horiz(small_H_first, zeros_2_first), arma::join_horiz(small_H_second, zeros_2_second));

            // Compute the covariance of the landmark in the map Ψ_k = H_k Σ H_k^T + R

            arma::mat psi_k{2,2, arma::fill::zeros};

            psi_k = H_k * sigma * H_k.t() + R;

            // Compute the expected measurement ^z_k = h(μ)

            double r_k_hat = std::sqrt(d_k);
            double phi_k_hat = normalize_angle(atan2(delta_k.y, delta_k.x) - q(0));

            arma::colvec z_k_hat{2,arma::fill::zeros};
            z_k_hat(0) = r_k_hat;
            z_k_hat(1) = phi_k_hat;

            // Compute the Mahalanobis distance D_k = (z_i − ^z_k)^T Ψ^{−1} (z_i − ^z_k)

            arma::colvec z_k_diff{2, arma::fill::zeros};
            z_k_diff(0) = z_i_temp(0) - z_k_hat(0);
            z_k_diff(1) = normalize_angle(z_i_temp(1) - z_k_hat(1));

            arma::colvec D_k{1, arma::fill::zeros};
            D_k = z_k_diff.t() * psi_k.i() * z_k_diff;

            // Calculate Euclidean distance
            arma::colvec eu_D_k{1, arma::fill::zeros};
            eu_D_k = z_k_diff.t() * z_k_diff;

            maha_distances.push_back(D_k);
            eu_distances.push_back(eu_D_k);
        }

        // Set Mahalanobis distance threshold to distance N+1
        double maha_benchmark = 0.0000004;
        // double maha_benchmark = 0.4;
        double maha_distance_threshold = std::max(maha_distances.at(maha_distances.size()-1)(0), maha_benchmark);
        double landmark_radius = 0.038;
        double eu_distance_threshold = (2.0) * landmark_radius + 0.5;
        // double eu_distance_threshold = (2.0) * landmark_radius;
        size_t index = N+1;
        bool new_landmark = true;

        if (N >= num_landmarks)
        {
            index = 0;
            new_landmark = false;
        }

        for (size_t j = 1; j <= maha_distances.size(); j++)
        {
            if (maha_distances.at(j-1)(0) < maha_distance_threshold) // Check for same landmark
            {
                maha_distance_threshold = std::max(maha_distances.at(j-1)(0), maha_benchmark);
                index = j;
                new_landmark = false; // Not a new landmark
            }
            else if (eu_distances.at(j-1)(0) < eu_distance_threshold) // Check for outliers
            {
                index = 0;
                new_landmark = false; // Not a new landmark
            }
        }

        if (new_landmark == true) // If it is a new landmark increase N
        {
            ++N;
        }

        return index;
    }

    // /// \brief 
    // size_t EKFSlam::associate_index(Point2D measurement)
    // {
    //     // For each measurement z_i
    //     // Convert relative measurements to range-bearing
    //     // r_i = (x^2 + y^2)^0.5
    //     // ϕ_i = atan2(y, x)
    //     const double r_i = magnitude(Vector2D{measurement.x, measurement.y});
    //     const double phi_i = std::atan2(measurement.y, measurement.x);     

    //     arma::vec z_i_unknown{2, arma::fill::zeros};
    //     z_i_unknown(0) = r_i;
    //     z_i_unknown(1) = phi_i;

    //     // Sensor noise matrix
    //     R = arma::mat{2, 2, arma::fill::eye} * R_noise; 

    //     // Number of landmarks that are part of the map
    //     bool possible_new_landmark = true;
    //     const size_t N = seen_landmarks.size();
    //     if (N + 1 > num_landmarks)
    //     {
    //         possible_new_landmark = false;
    //     }

    //     // Create a temporary map with new temporary landmark added
    //     arma::colvec m_temp{2*(N+1), arma::fill::zeros};
        
    //     for (size_t seen_landmark_index = 0; l)
        
    //     m_temp = m;
    //     // The new landmark is added to the map based on our current estimate of the robot pose
    //     m_temp(2*N) = q(1) + r_i * cos(phi_i + q(0));
    //     m_temp(2*N + 1) = q(2) + r_i * sin(phi_i + q(0));

    //     std::vector<arma::colvec> distances{};
        
    //     // For each landmark k (including the temporarily added landmark)
    //     for(size_t k = 1; k <= N+1; k++)
    //     {
    //         Vector2D delta_k{m(2*(k-1)) - q(1), m(2*(k-1)+1) - q(2)};
    //         double d_k = std::pow(magnitude(delta_k), 2);

    //         // Compute H_k 

    //         arma::mat H_k{2, num_dof+2*num_landmarks, arma::fill::zeros};

    //         arma::mat small_H_first{2, num_dof, arma::fill::zeros}; // Dependence on pose
    //         arma::mat zeros_2_first{2, 2 * (k-1), arma::fill::zeros}; // Dependence on landmarks having smaller indices
    //         arma::mat small_H_second{2, 2, arma::fill::zeros}; // Dependence on sensed landmark
    //         arma::mat zeros_2_second{2, 2 * num_landmarks - 2 * k, arma::fill::zeros}; // Dependence on landmarks having larger indices

    //         small_H_first(0, 0) = 0.0;
    //         small_H_first(0, 1) = -delta_k.x / std::sqrt(d_k);
    //         small_H_first(0, 2) = -delta_k.y / std::sqrt(d_k);
    //         small_H_first(1, 0) = -1;
    //         small_H_first(1, 1) = delta_k.y / d_k;
    //         small_H_first(1, 2) = -delta_k.x / d_k;

    //         small_H_second(0, 0) = delta_k.x / std::sqrt(d_k);
    //         small_H_second(0, 1) = delta_k.y / std::sqrt(d_k);
    //         small_H_second(1, 0) = -delta_k.y / d_k;
    //         small_H_second(1, 1) = delta_k.x / d_k;

    //         H_k = arma::join_horiz(arma::join_horiz(small_H_first, zeros_2_first), arma::join_horiz(small_H_second, zeros_2_second));

    //         // Compute the covariance of the landmark in the map Ψ_k = H_k Σ H_k^T + R

    //         arma::mat psi_k{2,2, arma::fill::zeros};

    //         psi_k = H_k * sigma * H_k.t() + R;

    //         // Compute the expected measurement ^z_k = h(μ)

    //         double r_k_hat = std::sqrt(d_k);
    //         double phi_k_hat = normalize_angle(atan2(delta_k.y, delta_k.x) - q(0));

    //         arma::colvec z_k_hat{2,arma::fill::zeros};
    //         z_k_hat(0) = r_k_hat;
    //         z_k_hat(1) = phi_k_hat;

    //         // Compute the Mahalanobis distance D_k = (z_i − ^z_k)^T Ψ^{−1} (z_i − ^z_k)

    //         arma::colvec z_k_diff{2, arma::fill::zeros};
    //         z_k_diff(0) = z_i(0) - z_k_hat(0);
    //         z_k_diff(1) = normalize_angle(z_i(1) - z_k_hat(1));

    //         arma::colvec D_k{1, arma::fill::zeros};
    //         D_k = z_k_diff.t() * psi_k.i() * z_k_diff;

    //         std::cout << "N: " << N <<  ", k: " << k << ", D_k: " << D_k(0) << "\n";

    //         distances.push_back(D_k);
    //     }

    //     // Include measurements that are close enough, and
    //     // Create a new landmark for measurements that are far enough
    //     const double inclusion_threshold = 0.3; // Translates to less than 7cm difference noise tolerance for small covariance
    //     const double newlandmark_threshold = 1.0; // Translates to around 10cm difference for new obstacle

    //     // Check mahalanobis distance between measurement and each mapped landmark
    //     for (size_t j = 1; j <= N; j++)
    //     {
    //         if (distances.at(j-1)(0) < inclusion_threshold) // Indicates landmark data
    //         {
    //             std::cout << "Landmark: " << j << "\n";
    //             return j;

    //             // return ((distances.at(1)(0))*100);
    //             // return distances.size();
    //         }
    //     }

    //     for (size_t j = 1; j <= N; j++)
    //     {
    //         if (distances.at(j-1)(0) < newlandmark_threshold) // Indicates outlier
    //         {
    //             std::cout << "Outlier: " << j << "\n";
    //             return 0;
    //         }
    //     }  

    //     // RCLCPP_ERROR("mahalanobis distances %f", distances.at(0)(0));
    //     // std::cout << "mahalanobis distance: " << distances.at(1)(0);
    //     std::cout << "New Landmark: " << N+1 << "\n";
    //     return N + 1; // Indicates new landmark
    //     // return ((distances.at(0)(0))*1e9);
    //     // return (0.01*1e9);
    // }

    /// GETTERS

    /// \brief get current pose prediction/correction of the robot
    Pose2D EKFSlam::pose() const
    {
        return Pose2D{q(0), q(1), q(2)};
    }

    /// \brief get current map vector prediction/correction of the robot
    arma::colvec EKFSlam::map() const
    {
        return m;
    }

    /// \brief get current state vector prediction/correction of the robot
    arma::colvec EKFSlam::state_vector() const
    {
        return Xi;
    }

    /// \brief get current covariance matrix prediction/correction of the robot
    arma::mat EKFSlam::covariance_matrix() const
    {
        return sigma;
    }

    /// \brief get current twist input to the robot
    Twist2D EKFSlam::twist() const
    {
        return Twist2D{u(0), u(1), u(2)};
    }

    /// \brief get current state matrix prediction of the robot
    arma::mat EKFSlam::state_matrix() const
    {
        return A;
    }

    /// \brief set the initial state of the robot
    arma::mat EKFSlam::actual_measurement() const
    {
        return z_i;
    }

    /// \brief get the current actual sensor measurement 
    arma::mat EKFSlam::predicted_measurement() const
    {
        return z_i_hat;
    }

    /// \brief get the current predicted sensor matrix 
    arma::mat EKFSlam::sensor_matrix() const
    {
        return H_i;
    }

    /// \brief get the number of seen landmarks
    size_t EKFSlam::num_seen_landmarks() const
    {
        return seen_landmarks.size();
    }
    
}