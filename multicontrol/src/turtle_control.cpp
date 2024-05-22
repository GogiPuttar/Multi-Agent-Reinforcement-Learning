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
    auto num_robots_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto wheel_radius_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto track_width_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto motor_cmd_max_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto motor_cmd_per_rad_sec_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto encoder_ticks_per_rad_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto collision_radius_des = rcl_interfaces::msg::ParameterDescriptor{};

    num_robots_des.description = "number of agents";
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
    declare_parameter("num_robots", 0, num_robots_des);     // 1,2,3,..
    declare_parameter("wheel_radius", -1.0, wheel_radius_des);
    declare_parameter("track_width", -1.0, track_width_des);
    declare_parameter("motor_cmd_max", -1.0, motor_cmd_max_des);
    declare_parameter("motor_cmd_per_rad_sec", -1.0, motor_cmd_per_rad_sec_des);
    declare_parameter("encoder_ticks_per_rad", -1.0, encoder_ticks_per_rad_des);
    declare_parameter("collision_radius", -1.0, collision_radius_des);

    // Get params - Read params from yaml file that is passed in the launch file
    num_robots_ = get_parameter("num_robots").get_parameter_value().get<int>();
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

    for(int i = 0; i < num_robots_; i++)
    {
      // Create Diff Drive Object
      turtles_.push_back(turtlelib::DiffDrive(wheel_radius_, track_width_));

      // Publishers
      // Create color/wheel_cmd publisher (MCUs)
      wheel_cmd_publishers_.push_back(create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
        colors_.at(i) + "/wheel_cmd", 10));

      // Create color/joint_states publisher
      joint_states_publishers_.push_back(create_publisher<sensor_msgs::msg::JointState>(
        colors_.at(i) + "/joint_states", 10));
    }

    // Subscribers
    // Create color/cmd_vel subscribers
    cyan_cmdvel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cyan/cmd_vel", 10, std::bind(
        &turtle_control::cyan_cmdvel_callback, this,
        std::placeholders::_1));
    magenta_cmdvel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "magenta/cmd_vel", 10, std::bind(
        &turtle_control::magenta_cmdvel_callback, this,
        std::placeholders::_1));
    yellow_cmdvel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "yellow/cmd_vel", 10, std::bind(
        &turtle_control::yellow_cmdvel_callback, this,
        std::placeholders::_1));
    red_cmdvel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "red/cmd_vel", 10, std::bind(
        &turtle_control::red_cmdvel_callback, this,
        std::placeholders::_1));
    green_cmdvel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "green/cmd_vel", 10, std::bind(
        &turtle_control::green_cmdvel_callback, this,
        std::placeholders::_1));
    blue_cmdvel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "blue/cmd_vel", 10, std::bind(
        &turtle_control::blue_cmdvel_callback, this,
        std::placeholders::_1));

    // Create color/sensor_data subscribers
    cyan_sensordata_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "cyan/sensor_data", 10, std::bind(
        &turtle_control::cyan_sensordata_callback,
        this, std::placeholders::_1));
    magenta_sensordata_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "magenta/sensor_data", 10, std::bind(
        &turtle_control::magenta_sensordata_callback,
        this, std::placeholders::_1));
    yellow_sensordata_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "yellow/sensor_data", 10, std::bind(
        &turtle_control::yellow_sensordata_callback,
        this, std::placeholders::_1));
    red_sensordata_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data", 10, std::bind(
        &turtle_control::red_sensordata_callback,
        this, std::placeholders::_1));
    green_sensordata_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "green/sensor_data", 10, std::bind(
        &turtle_control::green_sensordata_callback,
        this, std::placeholders::_1));
    blue_sensordata_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "blue/sensor_data", 10, std::bind(
        &turtle_control::blue_sensordata_callback,
        this, std::placeholders::_1));
  }

private:
  // Variables
  int num_robots_;
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
  std::vector<turtlelib::DiffDrive> turtles_;
  nuturtlebot_msgs::msg::WheelCommands wheel_cmd_;
  sensor_msgs::msg::JointState joint_states_;
  std::vector<std::string> colors_ = {"cyan", "magenta", "yellow", "red", "green", "blue", "orange", "brown", "white"};

  // Create objects
  std::vector<rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr> wheel_cmd_publishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> joint_states_publishers_;
  // color/cmd_vel subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cyan_cmdvel_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr magenta_cmdvel_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr yellow_cmdvel_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr red_cmdvel_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr green_cmdvel_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr blue_cmdvel_subscriber_;
  // color/sensor_data subscirbers
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr cyan_sensordata_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr magenta_sensordata_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr yellow_sensordata_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr red_sensordata_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr green_sensordata_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr blue_sensordata_subscriber_;

  /// \brief color_cmd_vel topic callbacks
  void cyan_cmdvel_callback(const geometry_msgs::msg::Twist & msg)
  {
    cmdvel2wheelcmd(msg, 0);
  }
  void magenta_cmdvel_callback(const geometry_msgs::msg::Twist & msg)
  {
    cmdvel2wheelcmd(msg, 1);
  }
  void yellow_cmdvel_callback(const geometry_msgs::msg::Twist & msg)
  {
    cmdvel2wheelcmd(msg, 2);
  }
  void red_cmdvel_callback(const geometry_msgs::msg::Twist & msg)
  {
    cmdvel2wheelcmd(msg, 3);
  }
  void green_cmdvel_callback(const geometry_msgs::msg::Twist & msg)
  {
    cmdvel2wheelcmd(msg, 4);
  }
  void blue_cmdvel_callback(const geometry_msgs::msg::Twist & msg)
  {
    cmdvel2wheelcmd(msg, 5);
  }

  void cmdvel2wheelcmd(const geometry_msgs::msg::Twist & msg, const int turtle_idx)
  {
    body_twist_.omega = msg.angular.z;
    body_twist_.x = msg.linear.x;
    body_twist_.y = msg.linear.y;

    // Perform Inverse kinematics to get the wheel velocities from the twist
    del_wheel_angles_ = turtles_.at(turtle_idx).TwistToWheels(body_twist_);

    // Convert rad/sec to ticks
    wheel_cmd_.left_velocity = round(del_wheel_angles_.left / motor_cmd_per_rad_sec_);
    wheel_cmd_.right_velocity = round(del_wheel_angles_.right / motor_cmd_per_rad_sec_);

    // Limit max wheel command speed and publish wheel command
    wheel_cmd_.left_velocity = limit_wheel_vel(wheel_cmd_.left_velocity);
    wheel_cmd_.right_velocity = limit_wheel_vel(wheel_cmd_.right_velocity);
    wheel_cmd_publishers_.at(turtle_idx)->publish(wheel_cmd_);
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

  /// \brief Sensor_data topic callback
  void cyan_sensordata_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    sensordata2jointstates(msg, 0);
  }
  void magenta_sensordata_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    sensordata2jointstates(msg, 1);
  }
  void yellow_sensordata_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    sensordata2jointstates(msg, 2);
  }
  void red_sensordata_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    sensordata2jointstates(msg, 3);
  }
  void green_sensordata_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    sensordata2jointstates(msg, 4);
  }
  void blue_sensordata_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    sensordata2jointstates(msg, 5);
  }

  void sensordata2jointstates(const nuturtlebot_msgs::msg::SensorData & msg, const int turtle_idx)
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
    joint_states_publishers_.at(turtle_idx)->publish(joint_states_);
  }

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