/// \file
/// \brief The slam node subscribes to joint_states and publishes to the green/odom navigation
///        topic. The node handles the Data association and SLAM calculations of the green robot.
///
/// PARAMETERS:
///     \param body_id (std::string): The name of the body frame of the robot
///     \param odom_id (std::string): The name of the odometry frame
///     \param wheel_left (std::string): The name of the left wheel joint
///     \param wheel_right (std::string): The name of the right wheel joint
///     \param wheel_radius (double): The radius of the wheels [m]
///     \param track_width (double): The distance between the wheels [m]
///     \param obstacles.r (double): Radius of cylindrical obstacles [m]
///     \param obstacles.h (double): Height of cylindrical obstacles [m]
///
/// PUBLISHES:
///     \param /odom (nav_msgs::msg::Odometry): Odometry publisher
///     \param /green/path (nav_msgs::msg::Path): Create the green turtle's nav_msgs/Path for rviz
///                                              visualization
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): SLAM estimate marker obstacles that are
///                                                                displayed in Rviz
///
/// SUBSCRIBES:
///     \param /joint_states (sensor_msgs::msg::JointState): Subscribes joint states for green robot
///     \param /nusim/fake_sensor (visualization_msgs::msg::MarkerArray): Fake sensor circles
///                                                                       MarkerArray as published
///                                                                       by the simulator (NUSIM)
///
/// SERVERS:
///     \param /initial_pose (std_srvs::srv::Empty): Sets initial pose of the turtle
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts green turtle position
///                                                             relative to odom
///     \param tf_broadcaster_2_ (tf2_ros::TransformBroadcaster): Broadcasts map to odom from SLAM
///                                                               corrections to ensure green turtle
///                                                               is in correct positions

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

/// \brief The class subscribes to joint_states and publishes to the odom navigation
///        topic. It has an initial pose service to set the start position of the robot.
///        The node publishes the location of the green robot that represents the SLAM
///        calculations.
///
///  \param body_id_ (std::string): The name of the body frame of the robot
///  \param odom_id_ (std::string): The name of the odometry frame
///  \param wheel_left_ (std::string): The name of the left wheel joint
///  \param wheel_right_ (std::string): The name of the right wheel joint
///  \param wheel_radius_ (double): The radius of the wheels [m]
///  \param track_width_ (double): The distance between the wheels [m]
///  \param obstacles_r_ (double): Radius of cylindrical obstacles [m]
///  \param basic_sensor_variance_ (double): Sensor noise variance [m]
///  \param max_range_ (double): Maximum range of sensing [m]

class slam : public rclcpp::Node
{
public:
  slam()
  : Node("slam")
  {
    // Parameter descirption
    auto body_id_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto odom_id_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto wheel_left_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto wheel_right_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto wheel_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto track_width_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto obstacles_r_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto basic_sensor_variance_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto max_range_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto use_laser_des = rcl_interfaces::msg::ParameterDescriptor{};

    body_id_des.description = "The name of the body frame of the robot";
    odom_id_des.description = "The name of the odometry frame";
    wheel_left_des.description = "The name of the left wheel joint";
    wheel_right_des.description = "The name of the right wheel joint";
    wheel_radius_des.description = "The radius of the wheels [m]";
    track_width_des.description = "The distance between the wheels [m]";
    obstacles_r_des.description = "Radius of cylindrical obstacles [m]";
    basic_sensor_variance_des.description = "Variance in landmark sensing [m^2]";
    max_range_des.description = "Range of landmark sensing [m]";
    use_laser_des.description = "Use circle fit on laser scan (true) or use fake sensor (false)";

    // Declare default parameters values
    declare_parameter("body_id", "green/base_footprint", body_id_des);
    declare_parameter("odom_id", "green/odom", odom_id_des);
    declare_parameter("wheel_left", "green/wheel_left_link", wheel_left_des);
    declare_parameter("wheel_right", "green/wheel_right_link", wheel_right_des);
    declare_parameter("wheel_radius", -1.0, wheel_radius_des);
    declare_parameter("track_width", -1.0, track_width_des);
    declare_parameter("obstacles.r", -1.0, obstacles_r_des);
    declare_parameter("basic_sensor_variance", -1.0, basic_sensor_variance_des); // Meters^2
    declare_parameter("max_range", -1.0, max_range_des); // Meters
    declare_parameter("use_laser", false, use_laser_des);

    // Get params - Read params from yaml file that is passed in the launch file
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").get_parameter_value().get<double>();
    max_range_ = get_parameter("max_range").get_parameter_value().get<double>();
    use_laser_ = get_parameter("use_laser").get_parameter_value().get<bool>();

    // Ensures all values are passed via the launch file
    check_frame_params();

    // Ensures all values are passed via .yaml file
    check_yaml_params();

    // Update object with params
    odom_turtle_ = turtlelib::DiffDrive{wheel_radius_, track_width_};
    // Extended Kalman Filter SLAM state estimator
    estimator_ptr_ = std::make_unique<turtlelib::EKFSlam>(odom_turtle_.pose());

    // Publishers
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "green/odom", 10);
    green_path_publisher_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);
    obstacles_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

    // Subscribers
    joint_states_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(
        &slam::joint_states_callback,
        this, std::placeholders::_1));

    if (use_laser_)
    {
      circle_fit_subscriber_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/landmarks/circle_fit", 10, std::bind(
          &slam::circle_fit_callback,
          this, std::placeholders::_1));
    }
    else
    {
      fake_sensor_subscriber_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", 10, std::bind(
          &slam::fake_sensor_callback,
          this, std::placeholders::_1));
    }

    // Initial pose service
    initial_pose_server_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(
        &slam::initial_pose_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster_2_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  // Variables
  std::string body_id_ = "";
  std::string odom_id_ = "";
  std::string wheel_left_ = "";
  std::string wheel_right_ = "";
  double wheel_radius_ = -1.0;
  double track_width_ = -1.0;
  double obstacles_r_ = -1.0;    // Size of obstacles
  double obstacles_h_ = 0.25;
  bool Flag_obstacle_seen_ = false;
  int readings_ = 0;
  turtlelib::wheelAngles del_wheel_angles_;
  turtlelib::wheelAngles prev_wheel_angles_;
  turtlelib::Twist2D body_twist_;
  tf2::Quaternion body_q_;
  tf2::Quaternion body_q2_;
  turtlelib::DiffDrive odom_turtle_;
  std::unique_ptr<turtlelib::EKFSlam> estimator_ptr_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::TransformStamped tf_;
  geometry_msgs::msg::TransformStamped tf2_;
  sensor_msgs::msg::JointState joint_states_;
  geometry_msgs::msg::PoseStamped green_pose_stamped_;
  nav_msgs::msg::Path green_path_;
  turtlelib::Pose2D green_turtle_{};
  turtlelib::Transform2D T_map_green{};
  turtlelib::Transform2D T_odom_green{};
  turtlelib::Transform2D T_map_odom{};
  int path_frequency_ = 100;
  turtlelib::Transform2D slam_tf_now_{};
  turtlelib::Transform2D slam_tf_prev_{};
  double basic_sensor_variance_ = -1.0;
  double max_range_ = -1.0;
  bool use_laser_ = false;

  // Create objects
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr circle_fit_subscriber_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_2_;

  /// \brief joint_states_callback subscription
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  {
    del_wheel_angles_.left = msg.position.at(0) - prev_wheel_angles_.left;
    del_wheel_angles_.right = msg.position.at(1) - prev_wheel_angles_.right;

    prev_wheel_angles_.left = msg.position.at(0);
    prev_wheel_angles_.right = msg.position.at(1);

    // Rely on wheel odometry until slam update.
    body_twist_ = odom_turtle_.driveWheels(del_wheel_angles_);
    body_q_.setRPY(0.0, 0.0, odom_turtle_.pose().theta);    

    // Publish odometry
    odometry_pub();

    // Broadcast transforms
    broadcast_map_odom_transform();
    broadcast_transforms();

    // SLAM estimate of landmarks
    create_obstacles_array();

    // Publish Nav Path
    readings_++;
    if (readings_ % path_frequency_ == 1) {
      update_green_NavPath();
      green_path_publisher_->publish(green_path_);
    }
  }

  /// \brief Fake landmark sensor topic callback
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    // Relative transform between successive sensing
    slam_tf_now_ = turtlelib::Transform2D{turtlelib::Vector2D{odom_turtle_.pose().x, odom_turtle_.pose().y}, odom_turtle_.pose().theta};
    turtlelib::Transform2D slam_tf_change_ = slam_tf_prev_.inv() * slam_tf_now_;

    // Twist that gets us from the previous sensor callback tf to this one.
    estimator_ptr_->predict(turtlelib::differentiate_transform(slam_tf_change_));

    slam_tf_prev_ = slam_tf_now_;

    visualization_msgs::msg::MarkerArray sensed_landmarks = msg;

    // Correct for each sensor measurement
    // j = 1, 2, 3...
    for (size_t j = 1; j <= sensed_landmarks.markers.size() - 0; j++) 
    {
      // Only use landmarks that the sensor currently sees
      if (sensed_landmarks.markers[j-1].action == visualization_msgs::msg::Marker::ADD) 
      { 
        estimator_ptr_->correct(sensed_landmarks.markers[j-1].pose.position.x, sensed_landmarks.markers[j-1].pose.position.y, j);
      }
    }
  }

  /// \brief Circle fit landmark sensor topic callback
  void circle_fit_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    // Relative transform between successive sensing
    slam_tf_now_ = turtlelib::Transform2D{turtlelib::Vector2D{odom_turtle_.pose().x, odom_turtle_.pose().y}, odom_turtle_.pose().theta};
    turtlelib::Transform2D slam_tf_change_ = slam_tf_prev_.inv() * slam_tf_now_;

    // Twist that gets us from the previous sensor callback tf to this one.
    estimator_ptr_->predict(turtlelib::differentiate_transform(slam_tf_change_));

    slam_tf_prev_ = slam_tf_now_;

    visualization_msgs::msg::MarkerArray sensed_landmarks = msg;

    // Correct for each sensor measurement
    for (size_t index = 0; index < sensed_landmarks.markers.size(); index++) 
    {
      // Convert measurements in base_scan frame to footprint frame
      turtlelib::Point2D landmark_position{sensed_landmarks.markers[index].pose.position.x - 0.032*cos(green_turtle_.theta), sensed_landmarks.markers[index].pose.position.y - 0.032*sin(green_turtle_.theta)};

      // turtlelib::Pose2D lidar_pose_{turtle_.pose().theta, turtle_.pose().x - 0.032*cos(turtle_.pose().theta), turtle_.pose().y - 0.032*sin(turtle_.pose().theta)};

      // Associate recevied landmark with the correct index
      // j = 1, 2, 3...
      size_t j = estimator_ptr_->associate_index(turtlelib::Point2D{
        landmark_position.x,
        landmark_position.y});
            
      if (j != 0)
      {
        // Correct using the landmark's measurement and id
        estimator_ptr_->correct(landmark_position.x, landmark_position.y, j);
      }
    }
  }

  /// \brief Ensures all values are passed via the launch file
  void check_frame_params()
  {
    if (body_id_ == "" || wheel_left_ == "" || wheel_right_ == "") {
      throw std::runtime_error("Missing frame id's! body_id, wheel_left, wheel_right");
    }
  }

  /// \brief Ensures all values are passed via .yaml file
  void check_yaml_params()
  {
    if (  wheel_radius_ == -1.0 ||
          track_width_ == -1.0 ||
          basic_sensor_variance_ == -1.0 ||
          max_range_ == -1.0 ||
          obstacles_r_ == -1.0
          )
    {
      RCLCPP_DEBUG(this->get_logger(), "Param wheel_radius: %f", wheel_radius_);
      RCLCPP_DEBUG(this->get_logger(), "Param track_width: %f", track_width_);
      RCLCPP_DEBUG(this->get_logger(), "Param basic_sensor_variance: %f", basic_sensor_variance_);
      RCLCPP_DEBUG(this->get_logger(), "Param max_range: %f", max_range_);
      RCLCPP_DEBUG(this->get_logger(), "Param obstacles_r_: %f", obstacles_r_);
      
      throw std::runtime_error("Missing necessary parameters in diff_params.yaml!");
    }

    if (  wheel_radius_ <= 0.0 ||
          track_width_ <= 0.0 ||
          basic_sensor_variance_ < 0.0 ||
          max_range_ <= 0.0 ||
          obstacles_r_ <= 0.0
          )
    {
      RCLCPP_DEBUG(this->get_logger(), "Param wheel_radius: %f", wheel_radius_);
      RCLCPP_DEBUG(this->get_logger(), "Param track_width: %f", track_width_);
      RCLCPP_DEBUG(this->get_logger(), "Param basic_sensor_variance: %f", basic_sensor_variance_);
      RCLCPP_DEBUG(this->get_logger(), "Param max_range: %f", max_range_);
      RCLCPP_DEBUG(this->get_logger(), "Param obstacles_r_: %f", obstacles_r_);
      
      throw std::runtime_error("Incorrect params in diff_params.yaml!");
    }
  }

  /// \brief Initial pose service TODO: might not work for arbitrary values
  void initial_pose_callback(
    nuturtle_control::srv::InitialPose::Request::SharedPtr request,
    nuturtle_control::srv::InitialPose::Response::SharedPtr)
  {
    // Set pose to initial pose
    odom_turtle_ = turtlelib::DiffDrive{wheel_radius_, track_width_, turtlelib::wheelAngles{0.0, 0.0}, turtlelib::Pose2D{request->theta, request->x, request->y}};
  }

  /// \brief Broadcasts green robot's position
  void broadcast_transforms()
  {
    // Broadcast odom frame
    tf_.header.stamp = get_clock()->now();
    tf_.header.frame_id = odom_id_;
    tf_.child_frame_id = body_id_;
    tf_.transform.translation.x = odom_turtle_.pose().x;
    tf_.transform.translation.y = odom_turtle_.pose().y;
    tf_.transform.translation.z = 0.0; 
    tf_.transform.rotation.x = body_q_.x();
    tf_.transform.rotation.y = body_q_.y();
    tf_.transform.rotation.z = body_q_.z();
    tf_.transform.rotation.w = body_q_.w();
    tf_broadcaster_->sendTransform(tf_);
  }

  /// \brief Broadcasts transform between map and odom
  void broadcast_map_odom_transform()
  {
    green_turtle_ = estimator_ptr_->pose();
    T_map_green = {turtlelib::Vector2D{green_turtle_.x, green_turtle_.y}, green_turtle_.theta};
    T_odom_green = {turtlelib::Vector2D{odom_turtle_.pose().x, odom_turtle_.pose().y}, odom_turtle_.pose().theta};
    T_map_odom = T_map_green * T_odom_green.inv();

    // Broadcast TF frames
    tf2_.header.stamp = get_clock()->now();
    tf2_.header.frame_id = "map";
    tf2_.child_frame_id = odom_id_;
    tf2_.transform.translation.x = T_map_odom.translation().x;
    tf2_.transform.translation.y = T_map_odom.translation().y;
    tf2_.transform.translation.z = 0.0;     // Turtle only exists in 2D
    body_q2_.setRPY(0, 0, T_map_odom.rotation());       // Rotation around z-axis
    tf2_.transform.rotation.x = body_q2_.x();
    tf2_.transform.rotation.y = body_q2_.y();
    tf2_.transform.rotation.z = body_q2_.z();
    tf2_.transform.rotation.w = body_q2_.w();
    tf_broadcaster_2_->sendTransform(tf2_);
  }

  /// \brief Create obstacles MarkerArray as seen by SLAM and publish them to a topic to display them in Rviz
  void create_obstacles_array()
  {
    arma::colvec map_vector = estimator_ptr_->map();
    visualization_msgs::msg::MarkerArray obstacles_;

    for (int landmark = 0; landmark < estimator_ptr_->num_seen_landmarks(); landmark++) {
      // if (map_vector(landmark) != 0) {
        visualization_msgs::msg::Marker obstacle_;
        obstacle_.header.frame_id = "map";
        obstacle_.header.stamp = get_clock()->now();
        obstacle_.id = landmark;
        obstacle_.type = visualization_msgs::msg::Marker::CYLINDER;
        obstacle_.action = visualization_msgs::msg::Marker::ADD;
        obstacle_.pose.position.x = map_vector(2*landmark);
        obstacle_.pose.position.y = map_vector(2*landmark + 1);
        obstacle_.pose.position.z = obstacles_h_ / 2.0;
        obstacle_.pose.orientation.x = 0.0;
        obstacle_.pose.orientation.y = 0.0;
        obstacle_.pose.orientation.z = 0.0;
        obstacle_.pose.orientation.w = 1.0;
        obstacle_.scale.x = obstacles_r_ * 2.0;       // Diameter in x
        obstacle_.scale.y = obstacles_r_ * 2.0;       // Diameter in y
        obstacle_.scale.z = obstacles_h_;             // Height
        obstacle_.color.r = 0.0f;
        obstacle_.color.g = 1.0f;
        obstacle_.color.b = 0.0f;
        obstacle_.color.a = 1.0;
        obstacles_.markers.push_back(obstacle_);
        Flag_obstacle_seen_ = true;
      // }
    }
    if (Flag_obstacle_seen_ == true) {
      obstacles_publisher_->publish(obstacles_);
    }
  }

  /// \brief Create the green turtle's nav_msgs/Path
  void update_green_NavPath()
  {
    // Update ground truth green turtle path
    green_path_.header.stamp = get_clock()->now();
    green_path_.header.frame_id = "map"; // TODO was green/odom
    // Create new pose stamped
    green_pose_stamped_.header.stamp = get_clock()->now();
    green_pose_stamped_.header.frame_id = "map"; // 
    green_pose_stamped_.pose.position.x = T_map_green.translation().x; //turtle_.pose().x;
    green_pose_stamped_.pose.position.y = T_map_green.translation().y; //turtle_.pose().y;
    green_pose_stamped_.pose.position.z = 0.0;
    green_pose_stamped_.pose.orientation.x = body_q2_.x(); // q_.x();
    green_pose_stamped_.pose.orientation.y = body_q2_.y(); // q_.y();
    green_pose_stamped_.pose.orientation.z = body_q2_.z(); // q_.z();
    green_pose_stamped_.pose.orientation.w = body_q2_.w(); // q_.w();
    // Append pose stamped
    green_path_.poses.push_back(green_pose_stamped_);
    green_path_publisher_->publish(green_path_);
  }

  /// \brief Publishes the odometry to the odom topic
  void odometry_pub()
  {
    // Publish updated odometry
    odom_.header.frame_id = odom_id_;
    odom_.child_frame_id = body_id_;
    odom_.header.stamp = get_clock()->now();
    odom_.pose.pose.position.x = odom_turtle_.pose().x;
    odom_.pose.pose.position.y = odom_turtle_.pose().y;
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation.x = body_q_.x();
    odom_.pose.pose.orientation.y = body_q_.y();
    odom_.pose.pose.orientation.z = body_q_.z();
    odom_.pose.pose.orientation.w = body_q_.w();
    odom_.twist.twist.linear.x = body_twist_.x;
    odom_.twist.twist.linear.y = body_twist_.y;
    odom_.twist.twist.angular.z = body_twist_.omega;
    odom_publisher_->publish(odom_);
  }
};

/// \brief Main function for node create, error handle and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam>());
  rclcpp::shutdown();
  return 0;
}