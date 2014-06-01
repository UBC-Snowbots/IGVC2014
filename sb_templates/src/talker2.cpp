
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <sstream>

using namespace std; 
using namespace ros; 

//global strings for ros node and topic names
static const string NODE_NAME = "talker2";
static const string PUBLISH_TOPIC = "vision_vel";
const int MSG_QUEUE_SIZE = 20;

double steering = 1;
double throttle = 2;

int main(int argc, char **argv)
{
  
  init(argc, argv, NODE_NAME);
  NodeHandle n;
 // Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000);

  Publisher chatter_pub = n.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC, MSG_QUEUE_SIZE);


  ros::Rate loop_rate(10);

  int count = 0;
  geometry_msgs::Twist twist;
twist.linear.x = 0;
twist.linear.y = 0;
twist.linear.z = 0;
twist.angular.x = 0;
twist.angular.y = 0;
twist.angular.z = 0;

  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "Twist Message: " << count;
    msg.data = ss.str();

  chatter_pub.publish(twist);
  ROS_INFO("I Published: throttle - %f, steering - %f", throttle,steering);

    //ROS_INFO("%s", msg.data.c_str());

    //chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}

