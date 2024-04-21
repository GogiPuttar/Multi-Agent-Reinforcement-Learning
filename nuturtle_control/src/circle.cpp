/// \file
/// \brief The circle node publishes to cmd_vel topic a twist that makes the turtle drive in a
///        circle with a given radius and angular angular velocity
///
/// PARAMETERS:
///     \param frequency (int): Timer callback frequency [Hz]
///
/// PUBLISHES:
///     \param /cmd_vel (geometry_msgs::msg::Twist): Publishes a twist to the cmd_vel topic
///
/// SUBSCRIBES:
///     None
///
/// SERVERS:
///     \param /control (nuturtle_control::srv::Control): Sets desired twist value
///     \param /reverse (std_srvs::srv::Empty): Inverts twist
///     \param /stop (std_srvs::srv::Empty): Sets twist to zero and stops publishing to cmd_vel
///
/// CLIENTS:
///     None

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

/// \brief This class publishes a twist to cmd_vel in the timer_callback. The control service
///        calculates the twist corresponding to a given circle radius and angular velocity.
///        The reverse service inverts the twist. The stop service sets the twist to zero and
///        terminates the publishing to cmd_vel topic. The loop runs at a fixed frequency until
///        termination.
///
///  \param stopping_flag_ (bool): Flag to stop publishing to cmd_vel for stop service
///  \param body_twist_ (geometry_msgs::msg::Twist): Twist to move robot in a circle

class circle : public rclcpp::Node
{
public:
  circle()
  : Node("circle")
  {
    // Parameter descirption
    auto frequency_des = rcl_interfaces::msg::ParameterDescriptor{};
    frequency_des.description = "Timer callback frequency [Hz]";
    // Declare default parameters values
    declare_parameter("frequency", 100, frequency_des);       // Hz for timer_callback
    // Get params - Read params from yaml file that is passed in the launch file
    int frequency = get_parameter("frequency").get_parameter_value().get<int>();

    //Initialize Body Twist
    body_twist_.linear.x = 0.0;
    body_twist_.angular.z = 0.0;

    // Publishers
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);

    // Control server
    control_server_ = create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(&circle::control_callback, this, std::placeholders::_1, std::placeholders::_2));
    // Reverse server
    reverse_server_ = create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));
    // Stop server
    stop_server_ = create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / frequency),
      std::bind(&circle::timer_callback, this));
  }

private:
  // State variables
  bool stopped_ = false;
  bool twist_changed_ = true; // Keeping track of when twists are updated
  geometry_msgs::msg::Twist body_twist_;

  // Create objects
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  /// \brief control service callback, sets desired twist value
  void control_callback(
    nuturtle_control::srv::Control::Request::SharedPtr request,
    nuturtle_control::srv::Control::Response::SharedPtr)
  {
    body_twist_.linear.x = request->radius * request->velocity;
    body_twist_.angular.z = request->velocity;

    twist_changed_ = true;
    stopped_ = false;
    RCLCPP_DEBUG(this->get_logger(), "Circle started. Twist: linear.x:%f, angular.z:%f", body_twist_.linear.x, body_twist_.angular.z);
  }

  /// \brief reverse service callback, inverts twist
  void reverse_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    body_twist_.linear.x = -body_twist_.linear.x;
    body_twist_.angular.z = -body_twist_.angular.z;

    twist_changed_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Circle reverse. Twist: linear.x:%f, angular.z:%f", body_twist_.linear.x, body_twist_.angular.z);
  }

  /// \brief stop service callback, sets twist to zero and stops publishing to cmd_vel
  void stop_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    body_twist_.linear.x = 0.0;
    body_twist_.angular.z = 0.0;
    cmd_vel_publisher_->publish(body_twist_);

    twist_changed_ = true;
    stopped_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Stopped. Twist: linear.x:%f, angular.z:%f", body_twist_.linear.x, body_twist_.angular.z);
  }

  /// \brief Main timer loop
  void timer_callback()
  {    
    if (!stopped_)
    {
      if (true) // Change 'true' to 'twist_changed' to publish cmd_vel only once, instead of at timer frequency.
      {  
        cmd_vel_publisher_->publish(body_twist_);
        twist_changed_ = false;
      }
    }
  }
};

/// \brief Main function for node create and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<circle>());
  rclcpp::shutdown();
  return 0;
}