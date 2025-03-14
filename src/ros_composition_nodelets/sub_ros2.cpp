#include "ros_composition_nodelets/sub_ros2.hpp"

namespace ros_composition_nodelets
{

SubROS2::SubROS2(const rclcpp::NodeOptions& options)
    : rclcpp::Node("sub_ros2", options)
{
  m_sub_ = this->create_subscription<std_msgs::msg::String>(
      "hello_world", 10,
      std::bind(&SubROS2::cbSubHelloWorld, this, std::placeholders::_1));
}

void SubROS2::cbSubHelloWorld(const std_msgs::msg::String::UniquePtr msg)
{
  std::stringstream str;
  str << "I heard: '" << msg->data << "' (sub ptr: " << msg.get() << ")";

  RCLCPP_INFO(this->get_logger(), "%s", str.str().c_str());
}

}  // namespace ros_composition_nodelets

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ros_composition_nodelets::SubROS2)
