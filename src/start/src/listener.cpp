#include "ros/ros.h"
#include "std_msgs/String.h"
#include "start/ObjectVec.h"
#include "start/Object.h"
#include "start/Rect.h"

/*void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/

void chatterCallback(const start::ObjectVec::ConstPtr& msg)
{
  ROS_INFO("I heard %lu: [%d], [%d]",  msg->objects.size(), msg->objects[0].classid, msg->objects[1].classid);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
 
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}


