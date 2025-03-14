#include "ros_composition_nodelets/pub_ros2.hpp"

#include <chrono>

namespace ros_composition_nodelets
{

using namespace std::chrono_literals;

PubROS2::PubROS2() : rclcpp::Node("pub_ros2"), m_counter_(0)
{
  m_pub_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);
  m_timer_ = this->create_wall_timer(1s, std::bind(&PubROS2::cbTimer, this));
}

void PubROS2::cbTimer()
{
  std_msgs::msg::String::UniquePtr msg =
      std::make_unique<std_msgs::msg::String>();

  std::stringstream str;
  str << "Hello, world! (" << m_counter_++ << ") (pub ptr: " << msg.get()
      << ")";

  msg->data = str.str();

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());

  m_pub_->publish(std::move(msg));
}

}  // namespace ros_composition_nodelets
