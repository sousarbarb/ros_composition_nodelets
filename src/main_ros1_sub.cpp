#include <nodelet/loader.h>
#include <ros/ros.h>

#include <iostream>

#include "ros_composition_nodelets/sub_ros1.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_ros1");
  ros::NodeHandle nh_priv("~");

  nodelet::Loader manager(false);
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  manager.load(ros::this_node::getName(), "ros_composition_nodelets/SubROS1",
               remappings, my_argv);

  ros::spin();
  return 0;
}
