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
// #include "nuturtle_control/srv/initial_pose.hpp"
#include "multicontrol/srv/control.hpp"
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
    auto num_robots_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto frequency_des = rcl_interfaces::msg::ParameterDescriptor{};

    num_robots_des.description = "number of agents";
    frequency_des.description = "Timer callback frequency [Hz]";

    // Declare default parameters values
    declare_parameter("num_robots", 0, num_robots_des);     // 1,2,3,..
    declare_parameter("frequency", 100, frequency_des);       // Hz for timer_callback

    // Get params - Read params from yaml file that is passed in the launch file
    num_robots_ = get_parameter("num_robots").get_parameter_value().get<int>();
    int frequency = get_parameter("frequency").get_parameter_value().get<int>();

    // Publishers
    for (int i = 0; i < num_robots_; i++)
    {
      cmd_vel_publishers_.push_back(create_publisher<geometry_msgs::msg::Twist>(
        colors_.at(i) + "/cmd_vel", 10));

      //Initialize Body Twist
      body_twists_.push_back(geometry_msgs::msg::Twist{});
    }
    
    // Control server
    control_server_ = create_service<multicontrol::srv::Control>(
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
  int num_robots_;
  bool stopped_ = false;
  bool twist_changed_ = true; // Keeping track of when twists are updated
  std::vector<geometry_msgs::msg::Twist> body_twists_;
  std::vector<std::string> colors_ = {"cyan", "magenta", "yellow", "red", "green", "blue", "orange", "brown", "white"};

  // Create objects
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_publishers_;
  rclcpp::Service<multicontrol::srv::Control>::SharedPtr control_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  /// \brief control service callback, sets desired twist value
  void control_callback(
    multicontrol::srv::Control::Request::SharedPtr request,
    multicontrol::srv::Control::Response::SharedPtr)
  {
    for (int i = 0; i < num_robots_; i++)
    {
      body_twists_.at(i).linear.x = request->radius * request->velocity;
      body_twists_.at(i).angular.z = request->velocity;
    }
    twist_changed_ = true;
    stopped_ = false;
    RCLCPP_DEBUG(this->get_logger(), "Circle started. Twist: linear.x:%f, angular.z:%f", body_twists_.at(0).linear.x, body_twists_.at(0).angular.z);
  }

  /// \brief reverse service callback, inverts twist
  void reverse_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    for (int i = 0; i < num_robots_; i++)
    {
      body_twists_.at(i).linear.x = -body_twists_.at(i).linear.x;
      body_twists_.at(i).angular.z = -body_twists_.at(i).angular.z;
    }

    twist_changed_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Circle reverse. Twist: linear.x:%f, angular.z:%f", body_twists_.at(0).linear.x, body_twists_.at(0).angular.z);
  }

  /// \brief stop service callback, sets twist to zero and stops publishing to cmd_vel
  void stop_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    
    for (int i = 0; i < num_robots_; i++)
    {
      body_twists_.at(i).linear.x = 0.0;
      body_twists_.at(i).angular.z = 0.0;
      cmd_vel_publishers_.at(i)->publish(body_twists_.at(i));
    }
    twist_changed_ = true;
    stopped_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Stopped. Twist: linear.x:%f, angular.z:%f", body_twists_.at(0).linear.x, body_twists_.at(0).angular.z);
  }

  /// \brief Main timer loop
  void timer_callback()
  {    
    if (!stopped_)
    {
      if (true) // Change 'true' to 'twist_changed' to publish cmd_vel only once, instead of at timer frequency.
      {  
        for (int i = 0; i < num_robots_; i++)
        {
          cmd_vel_publishers_.at(i)->publish(body_twists_.at(i));
        }
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