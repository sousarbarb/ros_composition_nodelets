#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros_composition_nodelets
{

class SubROS2 : public rclcpp::Node
{
 protected:

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub_;

 public:

  SubROS2();

 protected:

  void cbSubHelloWorld(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace ros_composition_nodelets
