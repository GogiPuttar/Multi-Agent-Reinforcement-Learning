#include <sstream>
#include <cmath>
#include <memory>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include <armadillo>

using turtlelib::normalize_angle;
using turtlelib::deg2rad;
using turtlelib::rad2deg;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::Transform2D;
using turtlelib::wheelAngles;
using turtlelib::Pose2D;
using turtlelib::DiffDrive;
using turtlelib::EKFSlam;
using turtlelib::w;
using turtlelib::num_dof;
using turtlelib::num_landmarks;
using turtlelib::PI;
using Catch::Matchers::WithinAbs;

// Covariance matrix
arma::mat sigma_0_q = arma::zeros<arma::mat>(num_dof, num_dof); // We are absolutely sure about the initial pose
arma::mat sigma_0_m = arma::eye(2 * num_landmarks, 2 * num_landmarks) * 1e6; // Uncertainity in sensing, very high with no knowledge of obstacles
arma::mat zeros_12 = arma::zeros<arma::mat>(num_dof, 2*num_landmarks); // Zeros due to sensing and localization noise being independent
arma::mat zeros_21 = arma::zeros<arma::mat>(2*num_landmarks, num_dof); // Zeros due to sensing and localization noise being independent
arma::mat zeros_22 = arma::zeros<arma::mat>(2*num_landmarks, 2*num_landmarks); // 
arma::mat I = arma::eye(num_dof + 2*num_landmarks, num_dof + 2*num_landmarks); // 
arma::mat sigma_0 =
    arma::join_vert(
    arma::join_horiz(sigma_0_q, zeros_12), 
    arma::join_horiz(zeros_21, sigma_0_m));

// State Matrix
arma::mat A_0 = arma::eye(num_dof+2*num_landmarks, num_dof+2*num_landmarks);

// Predictable process Noise
arma::mat Q{arma::mat{num_dof,num_dof,arma::fill::eye}*w};
arma::mat Q_bar =
            arma::join_vert(
            arma::join_horiz(Q, zeros_12), 
            arma::join_horiz(zeros_21, zeros_22));

void ekf_check_pose(EKFSlam subject, Pose2D required_pose);
void ekf_check_map(EKFSlam subject, arma::vec required_map);
void ekf_check_state_vector(EKFSlam subject, arma::vec required_state_vector);
void ekf_check_covariance_matrix(EKFSlam subject, arma::mat required_covariance_matrix);
void ekf_check_twist(EKFSlam subject, Twist2D required_twist);
void ekf_check_state_matrix(EKFSlam subject, arma::mat required_state_matrix);
void ekf_check_actual_measurement(EKFSlam subject, arma::vec required_actual_measurement);
void ekf_check_predicted_measurement(EKFSlam subject, arma::vec required_predicted_measurement);
void ekf_check_sensor_matrix(EKFSlam subject, arma::mat required_sensor_matrix);

TEST_CASE( "Initialization works for EKFSLam", "[EKFSlam()]") 
{
    EKFSlam estimator;

    // Pose
    ekf_check_pose(estimator, Pose2D{0.0, 0.0, 0.0});

    // Map
    ekf_check_map(estimator, arma::zeros<arma::vec>(2*num_landmarks));

    // State vector
    ekf_check_state_vector(estimator, arma::zeros<arma::vec>(num_dof + 2*num_landmarks));

    // Covariance matrix
    ekf_check_covariance_matrix(estimator, sigma_0);

    // Twist
    ekf_check_twist(estimator, Twist2D{0.0, 0.0, 0.0});

    // State Matrix
    ekf_check_state_matrix(estimator, A_0);

}

TEST_CASE( "Initialization with pose works for EKFSLam", "[EKFSlam(Pose2D)]") 
{
    Pose2D pose{-69, 6.9, 4.20};
    EKFSlam estimator(pose);

    // Pose
    ekf_check_pose(estimator, pose);

    // Map
    ekf_check_map(estimator, arma::zeros<arma::vec>(2*num_landmarks));

    // State vector
    ekf_check_state_vector(estimator, arma::join_vert(arma::vec({pose.theta, pose.x, pose.y}), arma::zeros<arma::vec>(2*num_landmarks)));

    // Covariance matrix
    ekf_check_covariance_matrix(estimator, sigma_0);

    // Twist
    ekf_check_twist(estimator, Twist2D{0.0, 0.0, 0.0});

    // State Matrix
    ekf_check_state_matrix(estimator, A_0);

    
}

TEST_CASE( "Prediction works for EKFSLam", "[predict(Twist2D)]") 
{
    Pose2D pose{-69, 6.9, 4.20};

    std::unique_ptr<turtlelib::EKFSlam> estimator_ptr;
    estimator_ptr = std::make_unique<turtlelib::EKFSlam>(pose);

    // EKFSlam estimator(pose);

    // 1. Check for pure rotation
    Twist2D v1{3.1, 0, 0};
    estimator_ptr->predict(v1);

    // Pose
    ekf_check_pose((*estimator_ptr), Pose2D{-3.0681469282, 6.9, 4.20});
    // Map
    ekf_check_map((*estimator_ptr), arma::zeros<arma::vec>(2*num_landmarks));
    // State vector
    ekf_check_state_vector((*estimator_ptr), arma::join_vert(arma::vec({-3.0681469282, 6.9, 4.20}), arma::zeros<arma::vec>(2*num_landmarks)));
    // Twist
    ekf_check_twist((*estimator_ptr), v1);
    // State matrix
    arma::mat A_1 = A_0;
    ekf_check_state_matrix((*estimator_ptr), A_1);
    // Covariance matrix
    arma::mat sigma_1 = A_1 * sigma_0 * A_1.t() + Q_bar;
    ekf_check_covariance_matrix((*estimator_ptr), sigma_1);

    // 2. Check for pure translation
    Twist2D v2{0, -6.9, 0};
    estimator_ptr->predict(v2);

    // Pose
    ekf_check_pose((*estimator_ptr), Pose2D{-3.0681469282, 13.781398116933687, 4.706320013688507});
    // Map
    ekf_check_map((*estimator_ptr), arma::zeros<arma::vec>(2*num_landmarks));
    // State vector
    ekf_check_state_vector((*estimator_ptr), arma::join_vert(arma::vec({-3.0681469282, 13.781398116933687, 4.706320013688507}), arma::zeros<arma::vec>(2*num_landmarks)));
    // Twist
    ekf_check_twist((*estimator_ptr), v2);

    // State matrix
    arma::mat small_A_2 = arma::zeros<arma::mat>(num_dof, num_dof);
    small_A_2(1,0) = -0.506320013688507;
    small_A_2(2,0) = 6.881398116933686;
    arma::mat A_2 = I + arma::join_vert(
        arma::join_horiz(small_A_2, zeros_12), 
        arma::join_horiz(zeros_21, zeros_22));
    ekf_check_state_matrix((*estimator_ptr), A_2);

    // Covariance matrix
    arma::mat sigma_2 = A_2 * sigma_1 * A_2.t() + Q_bar;
    ekf_check_covariance_matrix((*estimator_ptr), sigma_2);

    // 3. Check for general twist
    Twist2D v3{0.42, 6.9, 0};
    estimator_ptr->predict(v3);

    // Pose
    ekf_check_pose((*estimator_ptr), Pose2D{-2.6481469282, 7.2053095338, 2.7907798217});
    // Map
    ekf_check_map((*estimator_ptr), arma::zeros<arma::vec>(2*num_landmarks));
    // State vector
    ekf_check_state_vector((*estimator_ptr), arma::join_vert(arma::vec({-2.6481469282, 7.2053095338, 2.7907798217}), arma::zeros<arma::vec>(2*num_landmarks)));
    // Twist
    ekf_check_twist((*estimator_ptr), v3);

    // State matrix
    arma::mat small_A_3 = arma::zeros<arma::mat>(num_dof, num_dof);
    small_A_3(1,0) = 1.915540192024633;
    small_A_3(2,0) = -6.576088583127138;
    arma::mat A_3 = I + arma::join_vert(
        arma::join_horiz(small_A_3, zeros_12), 
        arma::join_horiz(zeros_21, zeros_22));
    ekf_check_state_matrix((*estimator_ptr), A_3);

    // Covariance matrix
    arma::mat sigma_3 = A_3 * sigma_2 * A_3.t() + Q_bar;
    ekf_check_covariance_matrix((*estimator_ptr), sigma_3);

    // 4. Check for improper twist (y-component)
    Twist2D v4{0.26, -6.9, -4.2};
    REQUIRE_THROWS(estimator_ptr->predict(v4));
}

TEST_CASE( "Correction works for EKFSlam", "[correct(double, double, size_t)]") 
{
    Pose2D pose{0.0, 0.0, 0.0};

    std::unique_ptr<turtlelib::EKFSlam> estimator_ptr;
    estimator_ptr = std::make_unique<turtlelib::EKFSlam>(pose);

    // 1. Check for one-dimensional translation along x-axis
    Twist2D v1{0, -0.069, 0};
    estimator_ptr->predict(v1);
    estimator_ptr->correct(2.0 + 0.069, 0, 1);

    // Actual measurement
    arma::vec z_1 = arma::zeros<arma::vec>(2);
    z_1(0) = 2.0 + 0.069;
    z_1(1) = 0.0;
    ekf_check_actual_measurement((*estimator_ptr), z_1);

    // Predicted measurement
    arma::vec z_1_hat = arma::zeros<arma::vec>(2);
    z_1_hat(0) = 2.069;
    z_1_hat(1) = 0.0;
    ekf_check_predicted_measurement((*estimator_ptr), z_1_hat);

    // Sensor matrix
    arma::mat H_1 = arma::zeros<arma::mat>(2, num_dof+2*num_landmarks);

    H_1(0,0) = 0;
    H_1(0,1) = -1;
    H_1(0,2) = 0;
    H_1(1,0) = -1;
    H_1(1,1) = 0;
    H_1(1,2) = -1/2.069;

    H_1(0,3) = 1;
    H_1(0,4) = 0;
    H_1(1,3) = 0;
    H_1(1,4) = 1/2.069;
    
    ekf_check_sensor_matrix((*estimator_ptr), H_1);
}

TEST_CASE( "Data association works for EKFSlam", "[associate_index(Point2D)]") 
{
    Point2D first_landmark_coordinates{2.0, 0.0};
    Point2D second_landmark_coordinates{2.8, 0.0};
    Point2D third_landmark_coordinates{3.8, 0.0};
    Point2D fourth_landmark_coordinates{5.0, 0.0};
    
    Pose2D pose{0.0, 0.0, 0.0};

    std::unique_ptr<turtlelib::EKFSlam> estimator_ptr;
    estimator_ptr = std::make_unique<turtlelib::EKFSlam>(pose);

    // 1. Check for one-dimensional translation along x-axis
    Twist2D v1{0, -0.069, 0};
    estimator_ptr->predict(v1);

    // Data association

    // Preliminary Testing (Not essential)
    // estimator_ptr->correct(2.0 + 0.069, 0, 1);
    // size_t j = estimator_ptr->associate_index(Point2D{2.0 + 0.069 + 0.068, 0.0});
    // REQUIRE(j == 69);

    // First landmark
    size_t j_1 = estimator_ptr->associate_index(Point2D{first_landmark_coordinates.x + 0.069, first_landmark_coordinates.y});
    REQUIRE(j_1 == 1);
    estimator_ptr->correct(first_landmark_coordinates.x + 0.069, first_landmark_coordinates.y, j_1);

    // Noisy first landmark
    double noise_1 = 0.0001;
    size_t j_1n = estimator_ptr->associate_index(Point2D{first_landmark_coordinates.x + 0.069 + noise_1, first_landmark_coordinates.y});
    REQUIRE(j_1n == 1);
    estimator_ptr->correct(first_landmark_coordinates.x + 0.069 + noise_1, first_landmark_coordinates.y, j_1n);

    // Outlier 
    double noise_01 = 0.069;
    size_t j_01 = estimator_ptr->associate_index(Point2D{first_landmark_coordinates.x + 0.069 + noise_01, first_landmark_coordinates.y});
    REQUIRE(j_01 == 0);

    // Second landmark
    size_t j_2 = estimator_ptr->associate_index(Point2D{second_landmark_coordinates.x + 0.069, second_landmark_coordinates.y});
    REQUIRE(j_2 == 2);
    estimator_ptr->correct(second_landmark_coordinates.x + 0.069, second_landmark_coordinates.y, j_2);

    // Noisy second landmark
    double noise_2 = 0.0001;
    size_t j_2n = estimator_ptr->associate_index(Point2D{second_landmark_coordinates.x + 0.069 + noise_2, second_landmark_coordinates.y});
    REQUIRE(j_2n == 2);
    estimator_ptr->correct(second_landmark_coordinates.x + 0.069 + noise_2, second_landmark_coordinates.y, j_2n);
    
    // Outlier
    double noise_02 = 0.001;
    size_t j_02 = estimator_ptr->associate_index(Point2D{second_landmark_coordinates.x + 0.069 + noise_02, second_landmark_coordinates.y});
    REQUIRE(j_02 == 0);

    // Third landmark
    size_t j_3 = estimator_ptr->associate_index(Point2D{third_landmark_coordinates.x + 0.069, third_landmark_coordinates.y});
    REQUIRE(j_3 == 3);
    estimator_ptr->correct(third_landmark_coordinates.x + 0.069, 0, j_3);

    // Noisy third landmark
    double noise_3 = 0.0001;
    size_t j_3n = estimator_ptr->associate_index(Point2D{third_landmark_coordinates.x + 0.069 + noise_3, third_landmark_coordinates.y});
    REQUIRE(j_3n == 3);
    estimator_ptr->correct(third_landmark_coordinates.x + 0.069 + noise_3, third_landmark_coordinates.y, j_3n);
    
    // Outlier
    double noise_03 = 0.001;
    size_t j_03 = estimator_ptr->associate_index(Point2D{third_landmark_coordinates.x + 0.069 + noise_03, third_landmark_coordinates.y});
    REQUIRE(j_03 == 0);

    // Fourth landmark
    size_t j_4 = estimator_ptr->associate_index(Point2D{fourth_landmark_coordinates.x + 0.069, fourth_landmark_coordinates.y});
    REQUIRE(j_4 == 4);
    estimator_ptr->correct(fourth_landmark_coordinates.x + 0.069, 0, j_4);

    // Extra landmark which can't be accomodated
    size_t j_5 = estimator_ptr->associate_index(Point2D{6.9 + 0.069, 0.0});
    REQUIRE(j_5 == 0);

    // Noisy third landmark again
    size_t j_3n2 = estimator_ptr->associate_index(Point2D{third_landmark_coordinates.x + 0.069 + noise_3, third_landmark_coordinates.y});
    REQUIRE(j_3n2 == 3);
    estimator_ptr->correct(third_landmark_coordinates.x + 0.069 + noise_3, third_landmark_coordinates.y, j_3n2);

    // /// 
    // estimator_ptr->correct(2.0 + 0.069, 0, 1);

    // // Actual measurement
    // arma::vec z_1 = arma::zeros<arma::vec>(2);
    // z_1(0) = 2.0 + 0.069;
    // z_1(1) = 0.0;
    // ekf_check_actual_measurement((*estimator_ptr), z_1);

    // // Predicted measurement
    // arma::vec z_1_hat = arma::zeros<arma::vec>(2);
    // z_1_hat(0) = 2.069;
    // z_1_hat(1) = 0.0;
    // ekf_check_predicted_measurement((*estimator_ptr), z_1_hat);

    // // Sensor matrix
    // arma::mat H_1 = arma::zeros<arma::mat>(2, num_dof+2*num_landmarks);

    // H_1(0,0) = 0;
    // H_1(0,1) = -1;
    // H_1(0,2) = 0;
    // H_1(1,0) = -1;
    // H_1(1,1) = 0;
    // H_1(1,2) = -1/2.069;

    // H_1(0,3) = 1;
    // H_1(0,4) = 0;
    // H_1(1,3) = 0;
    // H_1(1,4) = 1/2.069;
    
    // ekf_check_sensor_matrix((*estimator_ptr), H_1);
}



void ekf_check_pose(EKFSlam subject, Pose2D required_pose)
{
    REQUIRE_THAT( subject.pose().theta, WithinAbs(required_pose.theta,1.0e-6));
    REQUIRE_THAT( subject.pose().x, WithinAbs(required_pose.x,1.0e-6));
    REQUIRE_THAT( subject.pose().y, WithinAbs(required_pose.y,1.0e-6));
}

void ekf_check_map(EKFSlam subject, arma::vec required_map)
{
    REQUIRE( arma::approx_equal(subject.map(), required_map, "reldiff", 1e-6));
}

void ekf_check_state_vector(EKFSlam subject, arma::vec required_state_vector)
{
    REQUIRE( arma::approx_equal(subject.state_vector(), required_state_vector, "reldiff", 1e-6));
}

void ekf_check_covariance_matrix(EKFSlam subject, arma::mat required_covariance_matrix)
{
    // REQUIRE_THAT( subject.covariance_matrix()(1,0), WithinAbs(required_covariance_matrix(1,0),1.0e-6));
    REQUIRE( arma::approx_equal(subject.covariance_matrix(), required_covariance_matrix, "reldiff", 1e-6));
    REQUIRE( arma::approx_equal(subject.covariance_matrix().submat(0, num_dof, num_dof - 1, num_dof + 2*num_landmarks - 1), zeros_12, "reldiff", 1e-6));
    REQUIRE( arma::approx_equal(subject.covariance_matrix().submat(num_dof, 0, num_dof + 2*num_landmarks - 1, num_dof - 1), zeros_21, "reldiff", 1e-6));
}

void ekf_check_twist(EKFSlam subject, Twist2D required_twist)
{
    REQUIRE_THAT( subject.twist().omega, WithinAbs(required_twist.omega,1.0e-6));
    REQUIRE_THAT( subject.twist().x, WithinAbs(required_twist.x,1.0e-6));
    REQUIRE_THAT( subject.twist().y, WithinAbs(required_twist.y,1.0e-6));
}

void ekf_check_state_matrix(EKFSlam subject, arma::mat required_state_matrix)
{
    REQUIRE( arma::approx_equal(subject.state_matrix(), required_state_matrix, "reldiff", 1e-6));
}

void ekf_check_actual_measurement(EKFSlam subject, arma::vec required_actual_measurement)
{
    REQUIRE( arma::approx_equal(subject.actual_measurement(), required_actual_measurement, "reldiff", 1e-6));
}

void ekf_check_predicted_measurement(EKFSlam subject, arma::vec required_predicted_measurement)
{
    // REQUIRE_THAT( subject.predicted_measurement()(0), WithinAbs(required_predicted_measurement(0),1.0e-6));
    REQUIRE( arma::approx_equal(subject.predicted_measurement(), required_predicted_measurement, "reldiff", 1e-6));
}

void ekf_check_sensor_matrix(EKFSlam subject, arma::mat required_sensor_matrix)
{

    for(int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            // REQUIRE_THAT( subject.sensor_matrix()(j,i), WithinAbs(69.0,1.0e-6));
            REQUIRE_THAT( subject.sensor_matrix()(j,i), WithinAbs(required_sensor_matrix(j,i),1.0e-6));
        }
    }

    REQUIRE_THAT( subject.sensor_matrix().size(), WithinAbs(required_sensor_matrix.size(),1.0e-6));
    
    REQUIRE( arma::approx_equal(subject.sensor_matrix(), required_sensor_matrix, "absdiff", 1e-6));
}
