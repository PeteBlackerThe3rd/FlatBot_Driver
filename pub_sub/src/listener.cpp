#include "ros/ros.h"
#include "flat_bot_msgs/ModuleStatus.h"

void chatterCallback(const flat_bot_msgs::ModuleStatus::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->bus_voltage);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
