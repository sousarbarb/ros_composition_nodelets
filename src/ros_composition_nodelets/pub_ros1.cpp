#include "ros_composition_nodelets/pub_ros1.hpp"

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace ros_composition_nodelets
{

void PubROS1::onInit()
{
  ros::NodeHandle& nh = this->getNodeHandle();
  // ros::NodeHandle& nh_priv = this->getPrivateNodeHandle();

  m_pub_ = nh.advertise<std_msgs::String>("hello_world", 10);
  m_timer_ =
      nh.createTimer(ros::Duration(1), std::bind(&PubROS1::cbTimer, this));
}

void PubROS1::cbTimer()
{
  boost::shared_ptr<std_msgs::String> msg =
      boost::make_shared<std_msgs::String>();

  std::stringstream str;
  str << "Hello, world! (" << m_counter_++ << ") (pub ptr: " << msg.get()
      << ")";

  msg->data = str.str();

  NODELET_INFO("Publishing: '%s'", msg->data.c_str());

  m_pub_.publish(msg);
}

}  // namespace ros_composition_nodelets

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ros_composition_nodelets::PubROS1, nodelet::Nodelet)
