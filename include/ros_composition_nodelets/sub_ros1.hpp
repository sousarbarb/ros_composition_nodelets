#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace ros_composition_nodelets
{

class SubROS1 : public nodelet::Nodelet
{
 protected:

  ros::Subscriber m_sub_;

 public:

  virtual void onInit();

 protected:

  void cbSubHelloWorld(const std_msgs::String::ConstPtr& msg);
};

}  // namespace ros_composition_nodelets
