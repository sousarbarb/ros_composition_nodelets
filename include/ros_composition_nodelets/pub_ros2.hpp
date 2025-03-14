#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros_composition_nodelets
{

class PubROS2 : public rclcpp::Node
{
 protected:

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_timer_;

  int m_counter_ = 0;

 public:

  PubROS2();

 protected:

  void cbTimer();
};

}  // namespace ros_composition_nodelets
