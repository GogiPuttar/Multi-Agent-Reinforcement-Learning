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
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"

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
///  \param turtle_ (turtlelib::DiffDrive): Diff_drive robot
///  \param wheel_cmd_ (nuturtlebot_msgs::msg::WheelCommands): Desired wheel command
///  \param joint_states_ (sensor_msgs::msg::JointState): Joint states for blue robot

class turtle_control : public rclcpp::Node
{
public:
  turtle_control()
  : Node("turtle_control")
  {
    // Parameter descirption
    auto wheel_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto track_width_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto motor_cmd_max_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto motor_cmd_per_rad_sec_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto encoder_ticks_per_rad_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto collision_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    wheel_radius_des.description = "The radius of the wheels [m]";
    track_width_des.description = "The distance between the wheels [m]";
    motor_cmd_max_des.description =
      "The motors are provided commands in the interval \
                                         [-motor_cmd_max, motor_cmd_max]";
    motor_cmd_per_rad_sec_des.description = "Each motor command 'tick' is X [radians/sec]";
    encoder_ticks_per_rad_des.description =
      "The number of encoder 'ticks' per radian \
                                                 [ticks/rad]";
    collision_radius_des.description =
      "This is a simplified geometry used for collision \
                                            detection [m]";

    // Declare default parameters values
    declare_parameter("wheel_radius", -1.0, wheel_radius_des);
    declare_parameter("track_width", -1.0, track_width_des);
    declare_parameter("motor_cmd_max", -1.0, motor_cmd_max_des);
    declare_parameter("motor_cmd_per_rad_sec", -1.0, motor_cmd_per_rad_sec_des);
    declare_parameter("encoder_ticks_per_rad", -1.0, encoder_ticks_per_rad_des);
    declare_parameter("collision_radius", -1.0, collision_radius_des);

    // Get params - Read params from yaml file that is passed in the launch file
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();

    // Ensures all values are passed via .yaml file
    check_yaml_params();

    // Create Diff Drive Object
    turtle_ = turtlelib::DiffDrive(wheel_radius_, track_width_);

    // Publishers
    wheel_cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10);
    joint_states_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);

    // Subscribers
    cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(
        &turtle_control::cmd_vel_callback, this,
        std::placeholders::_1));
    sensor_data_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(
        &turtle_control::sensor_data_callback,
        this, std::placeholders::_1));
  }

private:
  // Variables
  double wheel_radius_;
  double track_width_;
  double motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
  double prev_encoder_stamp_ = -1.0;
  int ticks_at_0_left = 0;
  int ticks_at_0_right = 0;
  turtlelib::Twist2D body_twist_;
  turtlelib::wheelAngles del_wheel_angles_;
  turtlelib::DiffDrive turtle_;
  nuturtlebot_msgs::msg::WheelCommands wheel_cmd_;
  sensor_msgs::msg::JointState joint_states_;

  // Create objects
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_subscriber_;

  /// \brief cmd_vel topic callback
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    body_twist_.omega = msg.angular.z;
    body_twist_.x = msg.linear.x;
    body_twist_.y = msg.linear.y;

    // Perform Inverse kinematics to get the wheel velocities from the twist
    del_wheel_angles_ = turtle_.TwistToWheels(body_twist_);

    // Convert rad/sec to ticks
    wheel_cmd_.left_velocity = round(del_wheel_angles_.left / motor_cmd_per_rad_sec_);
    wheel_cmd_.right_velocity = round(del_wheel_angles_.right / motor_cmd_per_rad_sec_);

    // Limit max wheel command speed and publish wheel command
    wheel_cmd_.left_velocity = limit_wheel_vel(wheel_cmd_.left_velocity);
    wheel_cmd_.right_velocity = limit_wheel_vel(wheel_cmd_.right_velocity);
    wheel_cmd_publisher_->publish(wheel_cmd_);
  }

  /// \brief Limits the wheel command velocity to the max wheel command velocity
  /// \param wheel_vel (double)
  /// \returns Clipped wheel velocity (double)
  double limit_wheel_vel(double wheel_vel)
  {
    if (wheel_vel > motor_cmd_max_) {
      return motor_cmd_max_;
    } else if (wheel_vel < -motor_cmd_max_) {
      return -motor_cmd_max_;
    } else {
      return wheel_vel;
    }
  }

// ############## Begin Citation [https://github.com/Marnonel6/EKF_SLAM_from_scratch/blob/main/nuturtle_control/src/turtle_control.cpp]
  /// \brief Sensor_data topic callback
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {    
    joint_states_.header.stamp = msg.stamp;
    joint_states_.name = {"wheel_left_joint", "wheel_right_joint"};

    if (prev_encoder_stamp_ == -1.0) 
    {
      joint_states_.position = {0.0, 0.0};
      joint_states_.velocity = {0.0, 0.0};

      ticks_at_0_left = msg.left_encoder;
      ticks_at_0_right = msg.right_encoder;
    } 
    else 
    {
      // Change in wheel angle from encoder ticks
      joint_states_.position = {static_cast<double>(msg.left_encoder - ticks_at_0_left) / encoder_ticks_per_rad_,
        static_cast<double>(msg.right_encoder - ticks_at_0_right) / encoder_ticks_per_rad_};

      double delta_t_ = msg.stamp.sec + msg.stamp.nanosec * 1e-9 - prev_encoder_stamp_;

      // Encoder ticks to rad/s
      joint_states_.velocity = {joint_states_.position.at(0) / delta_t_,
        joint_states_.position.at(1) / delta_t_};
    }
    prev_encoder_stamp_ = msg.stamp.sec + msg.stamp.nanosec * 1e-9;

    // Publish joint states
    joint_states_publisher_->publish(joint_states_);
  }
// ########### End Citation

  /// \brief Ensures all values are passed via .yaml file
  void check_yaml_params()
  {
    if (wheel_radius_ == -1.0 || track_width_ == -1.0 || motor_cmd_max_ == -1.0 ||
      motor_cmd_per_rad_sec_ == -1.0 || encoder_ticks_per_rad_ == -1.0 ||
      collision_radius_ == -1.0)
    {
      RCLCPP_DEBUG(this->get_logger(), "Param: %f", wheel_radius_);
      RCLCPP_DEBUG(this->get_logger(), "Param: %f", track_width_);
      RCLCPP_DEBUG(this->get_logger(), "Param: %f", motor_cmd_max_);
      RCLCPP_DEBUG(this->get_logger(), "Param: %f", motor_cmd_per_rad_sec_);
      RCLCPP_DEBUG(this->get_logger(), "Param: %f", encoder_ticks_per_rad_);
      RCLCPP_DEBUG(this->get_logger(), "Param: %f", collision_radius_);
      
      throw std::runtime_error("Missing parameters in diff_params.yaml!");
    }
  }
};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_control>());
  rclcpp::shutdown();
  return 0;
}