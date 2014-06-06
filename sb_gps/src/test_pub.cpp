#include <ros/ros.h>
#include "sb_gps/Coord.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;
using namespace ros;

static const string NODE_NAME = "sample_gps";
static const string GPS_DATA_TOPIC = "sample_coord";
//TODO something that measures where Avalanche's position is at atm.
double curr_lat = 200, curr_long = 190; // keep adding 2 or something every second or so

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n;
	Publisher gps_data_pub = n.advertise<sb_gps::Coord>(GPS_DATA_TOPIC, 20);
	Rate loop_rate(1); // one cycle per sec ?
	
	while (ok())
	{
		sb_gps::Coord msg;
		msg.latitude = curr_lat;
		msg.longitude = curr_long;		
		ROS_INFO("Curr Lat: %f. Curr long: %f.", msg.latitude, msg.longitude);
		gps_data_pub.publish(msg);
		spinOnce();
		loop_rate.sleep();
		curr_long++;
		//curr_lat++; // increase lat_long manually
	}
	return 0;
}



