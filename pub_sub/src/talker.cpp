#include "ros/ros.h"
#include "flat_bot_msgs/ModuleStatus.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<flat_bot_msgs::ModuleStatus>("chatter", 1000);
  ros::Rate loop_rate(10);

  int count = 0; // count how many messages sent
  while (ros::ok())
  {
    flat_bot_msgs::ModuleStatus msg; //message object
    msg.bus_voltage = 5;
    ROS_INFO("%f", msg.bus_voltage);
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
