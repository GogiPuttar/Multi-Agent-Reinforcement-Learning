#ifndef EKFSlam_INCLUDE_GUARD_HPP
#define EKFSlam_INCLUDE_GUARD_HPP
/// \file
/// \brief Extended Kalman Filter (SLAM).

#include <iosfwd>
#include <cmath>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include <unordered_set>

namespace turtlelib
{
    /// \brief maximum number of obstacles
    constexpr int num_landmarks=4;
    /// \brief size of robot state vector
    constexpr int num_dof=3;
    /// \brief process noise for robot motion, variance
    // constexpr double w = 0.01;
    constexpr double w = 0.9;
    /// \brief sensing noise on landmarks, variance
    constexpr double R_noise = 0.01; // For small sensor noise
    // constexpr double R_noise = 0.9; // For large sensor noise

    /// \brief Kinematics of a differential drive robot.
    class EKFSlam
    {
    private:
    // public:
        /// \brief State vector of the robot at time t. q ∈ num_dof x 1. Contains the robot's pose. [theta x y]^T
        arma::colvec q{};
        /// \brief State vector of the map at time t. m ∈ 2*num_landmarks x 1. Contains the coordinates of every landmark. [m_x1 m_y1 m_x2 m_y2 ... m_xn m_yn]^T
        arma::colvec m{};
        /// \brief State vector of the system at time t. ξ ∈ (num_dof + 2*num_landmarks) x 1. Combined state of robot and map. [q ; m]
        arma::colvec Xi{};
        /// \brief Covariance matrix. Σ ∈ (num_dof + 2*num_landmarks) x (num_dof + 2*num_landmarks). Off diagonal terms are zero because sensing noise and movement uncertainity are independent.
        arma::mat sigma{};
        /// \brief Given twist. u ∈ num_dof x 1. Measured / commanded input to the state transition. Y-component zero for diff drive robot.
        arma::colvec u{num_dof,arma::fill::zeros};
        // /// \brief Previous twist. 
        // Twist2D prev_twist{0.0,0.0,0.0};
        /// \brief Identity matrix. I ∈ (num_dof + 2*num_landmarks) x (num_dof + 2*num_landmarks). Used to calculate A.
        const arma::mat I{num_dof+2*num_landmarks,num_dof+2*num_landmarks,arma::fill::eye};
        /// \brief Linearized state transition matrix. A ∈ (num_dof + 2*num_landmarks) x (num_dof + 2*num_landmarks). A_t = g'(ξ_{t−1}, u_t)
        arma::mat A{num_dof+2*num_landmarks, num_dof+2*num_landmarks,arma::fill::eye};
        /// \brief Process noise for the robot motion, as variance. Q ∈ num_dof x num_dof.
        const arma::mat Q{arma::mat{num_dof,num_dof,arma::fill::eye}*w};
        /// \brief Previously seen landmark IDs, j: 1, 2, 3...
        std::unordered_set<int> seen_landmarks{};
        /// \brief Actual measurement at i. z_i ∈ 2 x 1. Relative r_j and phi_j bearing measurements of a landmarks.
        arma::colvec z_i{2,arma::fill::zeros};
        /// \brief Estimate measurement. ˆz_i ∈ 2 x 1. Relative ˆr_j and ˆphi_j bearing predictions of a landmarks, based on pose prediction.
        arma::colvec z_i_hat{2,arma::fill::zeros};
        /// \brief H matrix
        arma::mat H_i{2, num_dof+2*num_landmarks, arma::fill::zeros};
        /// \brief Kalman gain
        arma::mat K_i{3+2*num_landmarks, 2, arma::fill::zeros};
        /// \brief Sensor noise, as variance
        arma::mat R{2,2,arma::fill::eye};
        /// \brief Number of landmarks visible
        size_t N = 0;

    public:
        /// \brief start at origin and default the uncertainty
        EKFSlam();

        /// \brief set robot start config and default the uncertainty
        /// \param turtle_pose_0 - robot start pose
        explicit EKFSlam(Pose2D turtle_pose_0);

        /// \brief set the initial guess covariance matrix
        void initialize_covariance();

        /// \brief set the initial state of the robot
        /// \param turtle_pose_0 - robot start pose
        void initialize_pose(Pose2D turtle_pose_0);

        void update_state_vector();

        void update_pose_and_map();

        /// \brief predict/estimate the robot state and propogate the uncertainty
        /// \param twist - twist control at time t
        void predict(Twist2D twist);

        /// \brief correction calculations
        /// \param x - sensed landmark relative x-coordinate
        /// \param y - aensed landmark relative y-coordinate
        /// \param j - sensed landmark index j
        void correct(double x, double y, size_t j);

        /// \brief data association
        /// \param measurement - current measurement of unknown landmark
        /// \returns j - index of that landmark
        size_t associate_index(Point2D measurement);

        // GETTERS
        
        /// \brief get current pose prediction/correction of the robot
        Pose2D pose() const;

        /// \brief get current map vector prediction/correction of the robot
        arma::colvec map() const;

        /// \brief get current state vector prediction/correction of the robot
        arma::colvec state_vector() const;

        /// \brief get current covariance matrix prediction/correction of the robot
        arma::mat covariance_matrix() const;

        /// \brief get current twist input to the robot
        Twist2D twist() const;

        /// \brief get current state matrix prediction of the robot
        arma::mat state_matrix() const;

        /// \brief get the current actual sensor measurement 
        arma::mat actual_measurement() const;

        /// \brief get the predicted sensor measurement based on the pose prediction
        arma::mat predicted_measurement() const;

        /// \brief get the current predicted sensor matrix 
        arma::mat sensor_matrix() const;

        /// \brief get number of seen landmarks
        size_t num_seen_landmarks() const;
    };
}

#endif