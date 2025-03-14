#include <iostream>

#include "ros_composition_nodelets/pub_ros2.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros_composition_nodelets::PubROS2>());
  rclcpp::shutdown();

  return 0;
}
