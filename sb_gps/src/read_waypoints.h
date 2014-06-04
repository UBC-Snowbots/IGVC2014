#ifndef READ_WAYPOINTS
#define READ_WAYPOINTS

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;

struct waypoint 
{
	double long_x;
	double lat_y;
};

int GetWaypoint(double goal_x, double goal_y, double* list);
void CalculateDistance(); 
double* ReturnWaypoints();

#endif
