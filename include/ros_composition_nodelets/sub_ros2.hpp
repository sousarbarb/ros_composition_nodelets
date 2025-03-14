#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/visibility_control.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros_composition_nodelets
{

class SubROS2 : public rclcpp::Node
{
 protected:

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub_;

 public:

  RCLCPP_COMPONENTS_PUBLIC
  explicit SubROS2(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 protected:

  void cbSubHelloWorld(const std_msgs::msg::String::UniquePtr msg);
};

}  // namespace ros_composition_nodelets
