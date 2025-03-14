#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/String.h>

namespace ros_composition_nodelets
{

class PubROS1 : public nodelet::Nodelet
{
 protected:

  ros::Publisher m_pub_;
  ros::Timer m_timer_;

  int m_counter_ = 0;

 public:

  virtual void onInit();

 protected:

  void cbTimer();
};

}  // namespace ros_composition_nodelets
