#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include "sb_gps/Coord.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

// REMEMBER TO EDIT WAYPOINTS FILE DIRECTORY

using namespace std;

struct waypoint 
{
	double long_x;
	double lat_y;
};

// functions
int GetWaypoint();
void CalculateDistance(); 
double* ReturnWaypoints();
void CheckWaypointStatus();

static const string GPS_NODE_NAME = "temp"; // gps_node TODO 
static const string GPS_OUTPUT_TOPIC = "temp_output"; // gps_nav TODO
static const string GPS_INPUT_TOPIC = "sample_coord"; // fix TODO
bool isAtGoal = false, isFinished = false; // TODO
double lat, lon, goal_lat, goal_lon, dist, x_dist, y_dist;
struct waypoint current_waypoint;
struct waypoint goal_waypoint;
double* waypoints_list = ReturnWaypoints();
int size = sizeof(waypoints_list)*2, c = 0;
// var for checking last waypoint TODO

/*
// callback fnc TODO
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
// TODO:
	msg->status // NavSatStatus
	current_waypoint.lat_y = msg->latitude;
	current_waypoint.long_x = msg->longitude;
	ROS_INFO("\n>>GPS\nLat(y): %f\nLong(x): %f\n>>SAVED\nLat(y): %f\nLong(x): %f\n", msg->latitude, msg->longitude, current_waypoint.lat_y, current_waypoint.long_x);
}
*/

// temp callback
void gpsCallback(const sb_gps::Coord::ConstPtr& msg) 
{
	current_waypoint.lat_y = msg->latitude;
	current_waypoint.long_x = msg->longitude;
	ROS_INFO("\n>>GPS\nLat(y): %f\nLong(x): %f\n>>SAVED\nLat(y): %f\nLong(x): %f\n", msg->latitude, msg->longitude, current_waypoint.lat_y, current_waypoint.long_x);
}


int main(int argc, char **argv) 
{

	// hard code current waypoint 
	current_waypoint.long_x = 0; // keep like this until 
	current_waypoint.lat_y = 0;

	ros::init(argc, argv, GPS_NODE_NAME); // start node
	ros::NodeHandle nh; // main access point to communication with ROS system. First one initializes node.

	ros::Publisher gps_data_pub = nh.advertise<geometry_msgs::Twist>(GPS_OUTPUT_TOPIC, 20); // publisher for output
	ros::Subscriber gps_sub = nh.subscribe(GPS_INPUT_TOPIC, 20, gpsCallback); // TODO subscribe to gps data

	ros::Rate loop_rate(5); // 10hz loop rate

	int next_waypoint = GetWaypoint();

	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;

	while (ros::ok()) 
	{
		CalculateDistance();

		// calculated angular z
		if (x_dist / abs(x_dist) == -1) { msg.angular.z = -1; }
		else { msg.angular.z = 1; }

		msg.linear.x = x_dist;
		msg.linear.y = y_dist;
		gps_data_pub.publish(msg);
		ROS_INFO("\nLinear.x = %f\nLinear.y = %f\nAngular.z = %f\n", msg.linear.x, msg.linear.y, msg.angular.z);
		ros::spinOnce();
		loop_rate.sleep(); // sleep for 10hz
		CheckWaypointStatus();
		next_waypoint = GetWaypoint();
		if (isFinished) { return 0; }
	}
	return 0;
}


// Get the next waypoint: returns n for the nth waypoint, starting at 1
// return -1 for 'no more waypoints'
// void this function TODO
int GetWaypoint() 
{// TODO
	if (c >= size ) { isFinished = true; }
	if (c == 0 || isAtGoal) {
		goal_waypoint.long_x = waypoints_list[c];
		goal_waypoint.lat_y = waypoints_list[c+1];
		c += 2;
		isAtGoal = false;
		return (c+1)/2;
	}
}


// TODO Checks if we are sufficiently close to the waypoint
void CheckWaypointStatus() 
{
	// estimated to within 1.1m
	int cx, cy, gx, gy;
	cx = ceil(current_waypoint.long_x*10000)/10000;
	cy = ceil(current_waypoint.lat_y*10000)/10000;
	gx = ceil(goal_waypoint.long_x*10000)/10000;
	gy = ceil(goal_waypoint.lat_y*10000)/10000;
	if (cx == gx && cy == gy) { isAtGoal = true; }
	// something to check if it is the last waypoint TODO

}


// Calculates Euclidean distance from position to goal
void CalculateDistance()
{
	if (current_waypoint.long_x == 0 || current_waypoint.lat_y == 0) { dist = -1; }
	else {
		x_dist = goal_waypoint.long_x - current_waypoint.long_x;
		y_dist = goal_waypoint.lat_y - current_waypoint.lat_y;
		dist = sqrt(pow(x_dist, 2.0) + pow(y_dist, 2.0));
	}
}	


// Retrieves waypoints from the waypoints file.
double* ReturnWaypoints() 
{
	string output;
	int count = 0;
	int array_size;
	double* waypoints_array = NULL;
	ifstream waypoints_file ("/home/jechli/snowbots_ws/src/WaypointsTxt/waypoints.txt"); // TODO
	if (waypoints_file.is_open())
	{
		while (getline(waypoints_file, output))
		{
			if (count == 0) {
				array_size = atoi(output.c_str())*2;
				waypoints_array = new double [array_size];
				count++;
			}
			
			else { 
				waypoints_array[count-1] = atof(output.c_str());
				count++;
			}
		}
		waypoints_file.close();
	}

	else
	{
		ROS_INFO("Unable to open file");
	}

	return waypoints_array;
}	
    

