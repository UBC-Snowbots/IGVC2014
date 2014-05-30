#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;

static const string NODE_NAME = "gps_example_subscriber";
static const string GPS_OUTPUT_TOPIC = "sb_gps_output";

void gpsCallback(const std_msgs::String::ConstPtr& msg) 
{
	ROS_INFO("Got back: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe(GPS_OUTPUT_TOPIC, 1000, gpsCallback);
	ros::spin();
	return 0;
}
