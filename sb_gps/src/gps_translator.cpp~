#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <iostream>
#include <string>
#include <math.h>

static const string GPS_NODE_NAME = "gps_node";

static const string GPS_POSITION_TOPIC = "fix"; // gps: sub
static const string GPS_TIMESTAMP_TOPIC = "time_reference"; // timestamp of original gps reading: sub
static const string GPS_TIMEREAD_TOPIC = "nmea_sentence"; // timestamp read from robot: sub
static const string GPS_OUTPUT_TOPIC = "sb_gps_output"; // this node's output: pub

// for subscribers
void positionCallback(const TYPE msg)
{
	// do something TODO
}

void timeStampCallback(const TYPE msg)
{
	// do something TODO
}

void timeReadCallback(const TYPE msg)
{
	// do something TODO
}
 

int main(int argc, char **argv) 
{
	ros::init(argc, argv, GPS_NODE_NAME); // start node
	ros::NodeHandle nh; // main access point to communication with ROS system. First one initializes node.
	ros::Publisher gps_data_pub = nh.advertise<SOME SB_GPS_MSG>(GPS_OUTPUT_TOPIC, 1000); // publisher for output
	ros::Rate loop_rate(10); // 10hz loop rate
  
	int count = 0;
	while (ros::ok()) // false when Ctrl-C, kicked off network, ros::shutdown(), all Nodehandles destroyed.
	{
		// broadcast data TODO
		gps_data_pub.publish(msg);
		ROS_INFO("%s", msg.data.c_str()); // cout
		ros::spinOnce();
		loop_rate.sleep(); // sleep for 10hz
		++count;
	}
	return 0;
}

		
    

