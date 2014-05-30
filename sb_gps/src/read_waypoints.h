#ifndef READ_WAYPOINTS
#define READ_WAYPOINTS

#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>



struct waypoint 
{
	double long_x;
	double lat_y;
};

double* ReturnWaypoints();

#endif
