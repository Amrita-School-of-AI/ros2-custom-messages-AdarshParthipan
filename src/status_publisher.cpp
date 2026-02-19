#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// Include the header for our custom message
// Pattern is: <package_name>/msg/<snake_case_file_name>.hpp
#include "ros2_custom_msgs/msg/robot_status.hpp"

using namespace std::chrono_literals;

class StatusPublisher : public rclcpp::Node
{
public:
  StatusPublisher()
  : Node("status_publisher"), battery_level_(100.0), mission_count_(0)
  {
    // Create publisher for 'ros2_custom_msgs/msg/RobotStatus' on topic '/robot_status'
    publisher_ = this->create_publisher<ros2_custom_msgs::msg::RobotStatus>("/robot_status", 10);

    // Create a timer to fire every 1000ms (1 second)
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&StatusPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = ros2_custom_msgs::msg::RobotStatus();

    // Populate message fields per requirements
    message.robot_name = "Explorer1";
    message.battery_level = battery_level_;
    message.is_active = true;
    message.mission_count = mission_count_;

    // Log the status to console
    RCLCPP_INFO(this->get_logger(), "Publishing: robot=%s, battery=%.1f, active=%s, missions=%d",
      message.robot_name.c_str(),
      message.battery_level,
      message.is_active ? "true" : "false",
      message.mission_count);

    // Publish the message
    publisher_->publish(message);

    // Update state for next tick
    battery_level_ -= 0.5;
    mission_count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros2_custom_msgs::msg::RobotStatus>::SharedPtr publisher_;
  
  // Internal state variables
  double battery_level_;
  int32_t mission_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusPublisher>());
  rclcpp::shutdown();
  return 0;
}