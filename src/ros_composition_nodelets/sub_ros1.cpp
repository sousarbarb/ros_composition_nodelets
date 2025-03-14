#include "ros_composition_nodelets/sub_ros1.hpp"

namespace ros_composition_nodelets
{

void SubROS1::onInit()
{
  ros::NodeHandle& nh = this->getNodeHandle();
  // ros::NodeHandle& nh_priv = this->getPrivateNodeHandle();

  m_sub_ = nh.subscribe<std_msgs::String>("hello_world", 10,
                                          &SubROS1::cbSubHelloWorld, this);
}

void SubROS1::cbSubHelloWorld(const std_msgs::String::ConstPtr& msg)
{
  std::stringstream str;
  str << "I heard: '" << msg->data << "' (sub ptr: " << msg.get() << ")";

  NODELET_INFO("%s", str.str().c_str());
}

}  // namespace ros_composition_nodelets

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ros_composition_nodelets::SubROS1, nodelet::Nodelet)
