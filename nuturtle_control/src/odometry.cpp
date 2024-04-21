/// \file
/// \brief The turtle_control node handles the control of the physical/red robot.
///
/// PARAMETERS:
///     \param wheel_radius (double): The radius of the wheels [m]
///     \param track_width (double): The distance between the wheels [m]
///     \param motor_cmd_max (double): Maximum motor command value in ticks velocity
///     \param motor_cmd_per_rad_sec (double): Motor command to rad/s conversion factor
///     \param encoder_ticks_per_rad (double): Encoder ticks to radians conversion factor
///     \param collision_radius (double): Robot collision radius [m]
///
/// PUBLISHES:
///     \param /joint_states (sensor_msgs::msg::JointState): Publishes joint states for blue robot
///     \param /wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Wheel command value velocity in
///                                                               ticks
///
/// SUBSCRIBES:
///     \param /cmd_vel (geometry_msgs::msg::Twist): Command velocity twist
///     \param /sensor_data (nuturtlebot_msgs::msg::SensorData): This is the wheel encoder
///                                                              output in position ticks
///
/// SERVERS:
///     None
///
/// CLIENTS:
///     None

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

/// \brief The class subscribes to cmd_vel and converts the desired twist with inverse kinematics
///        into wheel commands and publishes it to the wheel_cmd topic. It subscribes to the
///        sensor_data and converts it to joint states for the robot and publishes it to the joint
///        states topic.
///
///  \param wheel_radius_ (double): The radius of the wheels [m]
///  \param track_width_ (double): The distance between the wheels [m]
///  \param motor_cmd_max_ (double): Maximum motor command value in ticks velocity
///  \param motor_cmd_per_rad_sec_ (double): Motor command to rad/s conversion factor
///  \param encoder_ticks_per_rad_ (double): Encoder ticks to radians conversion factor
///  \param collision_radius_ (double): Robot collision radius [m]
///  \param prev_encoder_stamp_ (double): Previous encoder time stamp
///  \param body_twist_ (turtlelib::Twist2D): Desired twist for robot
///  \param del_wheel_angles_ (turtlelib::wheelAngles): Wheel velocities
///  \param odom_turtle_ (turtlelib::DiffDrive): Diff_drive robot
///  \param wheel_cmd_ (nuturtlebot_msgs::msg::WheelCommands): Desired wheel command
///  \param joint_states_ (sensor_msgs::msg::JointState): Joint states for blue robot

class odometry : public rclcpp::Node
{
public:
  odometry()
  : Node("odometry")
  {
    // Parameter descirption
    auto body_id_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto odom_id_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto wheel_left_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto wheel_right_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto wheel_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto track_width_des = rcl_interfaces::msg::ParameterDescriptor{};

    body_id_des.description = "The name of the body frame of the robot";
    odom_id_des.description = "The name of the odometry frame";
    wheel_left_des.description = "The name of the left wheel joint frame";
    wheel_right_des.description = "The name of the right wheel joint frame";
    wheel_radius_des.description = "The radius of the wheels [m]";
    track_width_des.description = "The distance between the wheels [m]";

    // Declare default parameters values
    declare_parameter("body_id", "", body_id_des);
    declare_parameter("odom_id", "odom", odom_id_des);
    declare_parameter("wheel_left", "", wheel_left_des);
    declare_parameter("wheel_right", "", wheel_right_des);
    declare_parameter("wheel_radius", -1.0, wheel_radius_des);
    declare_parameter("track_width", -1.0, track_width_des);

    // Get params - Read params from yaml file that is passed in the launch file
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();

    // Ensures all values are passed via the launch file
    check_frame_params();
    
    // Ensures all values are passed via .yaml file
    check_yaml_params();

    // Create Diff Drive Object
    odom_turtle_ = turtlelib::DiffDrive(wheel_radius_, track_width_);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Publishers
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "odom", 10);

    blue_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Subscribers
    joint_states_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(
        &odometry::joint_states_callback, this,
        std::placeholders::_1));

    // Services
    initial_pose_server_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(
        &odometry::initial_pose_callback, this, std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  // Variables
  std::string body_id_ = "";
  std::string odom_id_ = "";
  std::string wheel_left_ = "";
  std::string wheel_right_ = "";
  double wheel_radius_ = -1.0;
  double track_width_ = -1.0;
  turtlelib::wheelAngles del_wheel_angles_;
  turtlelib::wheelAngles prev_wheel_angles_;
  turtlelib::Twist2D body_twist_;
  tf2::Quaternion body_q_;
  turtlelib::DiffDrive odom_turtle_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::TransformStamped tf_;
  sensor_msgs::msg::JointState joint_states_;
  int readings_ = 0;
  nav_msgs::msg::Path blue_path_;
  int path_frequency_ = 100;

  // Create objects
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_server_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blue_path_publisher_;
  geometry_msgs::msg::PoseStamped blue_pose_stamped_;

  /// \brief initial_pose_callback service
  void initial_pose_callback(
    nuturtle_control::srv::InitialPose::Request::SharedPtr request,
    nuturtle_control::srv::InitialPose::Response::SharedPtr
  )
  {
    odom_turtle_ = turtlelib::DiffDrive{wheel_radius_, track_width_, turtlelib::wheelAngles{0.0, 0.0}, turtlelib::Pose2D{request->theta, request->x, request->y}};
  }

  /// \brief joint_states_callback subscription
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  {
    del_wheel_angles_.left = msg.position.at(0) - prev_wheel_angles_.left;
    del_wheel_angles_.right = msg.position.at(1) - prev_wheel_angles_.right;

    // RCLCPP_INFO(this->get_logger(), "Param: %f", del_wheel_angles_.left);

    body_twist_ = odom_turtle_.driveWheels(del_wheel_angles_);
    body_q_.setRPY(0.0, 0.0, odom_turtle_.pose().theta);       

    odometry_pub();

    broadcast_transforms();

    prev_wheel_angles_.left = msg.position.at(0);
    prev_wheel_angles_.right = msg.position.at(1);

    readings_++;
    if (readings_ % path_frequency_ == 1) {
      update_blue_NavPath();
      blue_path_publisher_->publish(blue_path_);
    }
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
    odom_.twist.twist.angular.z = body_twist_.omega;
    odom_.twist.twist.linear.x = body_twist_.x;
    odom_.twist.twist.linear.y = body_twist_.y;
    odom_publisher_->publish(odom_);
  }

  /// \brief Broadcasts blue robot's position
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

  /// \brief Create the red turtle's nav_msgs/Path
  void update_blue_NavPath()
  {
    // Update odometry blue turtle path
    blue_path_.header.stamp = get_clock()->now();
    blue_path_.header.frame_id = odom_id_;
    // Create new pose stamped
    blue_pose_stamped_.header.stamp = get_clock()->now();
    blue_pose_stamped_.header.frame_id = odom_id_;
    blue_pose_stamped_.pose.position.x = odom_turtle_.pose().x;
    blue_pose_stamped_.pose.position.y = odom_turtle_.pose().y;
    blue_pose_stamped_.pose.position.z = 0.0;
    tf2::Quaternion body_q_;
    body_q_.setRPY(0, 0, odom_turtle_.pose().theta);     // Rotation around z-axis
    blue_pose_stamped_.pose.orientation.x = body_q_.x();
    blue_pose_stamped_.pose.orientation.y = body_q_.y();
    blue_pose_stamped_.pose.orientation.z = body_q_.z();
    blue_pose_stamped_.pose.orientation.w = body_q_.w();
    // Append pose stamped
    blue_path_.poses.push_back(blue_pose_stamped_);
  }

  /// \brief Ensures all values are passed via the launch file
  void check_frame_params()
  {
    if (body_id_ == "" || wheel_left_ == "" || wheel_right_ == "")
    {      
      throw std::runtime_error("Missing body_id, wheel_left, or wheel_right frames!");
    }
  }

  /// \brief Ensures all values are passed via .yaml file
  void check_yaml_params()
  {
    if (
          wheel_radius_ == -1.0 ||
          track_width_ == -1.0
        )
    {
      RCLCPP_DEBUG(this->get_logger(), "Param: %f", wheel_radius_);
      RCLCPP_DEBUG(this->get_logger(), "Param: %f", track_width_);
      
      throw std::runtime_error("Missing parameters in diff_params.yaml!");
    }
    if (  wheel_radius_ <= 0.0 ||
          track_width_ <= 0.0
        )
    {
      RCLCPP_DEBUG(this->get_logger(), "Param wheel_radius: %f", wheel_radius_);
      RCLCPP_DEBUG(this->get_logger(), "Param track_width: %f", track_width_);
      
      throw std::runtime_error("Incorrect params in diff_params.yaml!");
    }
  }
};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odometry>());
  rclcpp::shutdown();
  return 0;
}