#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
//#include "read_waypoints.h"

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

static const string GPS_NODE_NAME = "gps_node";
static const string GPS_OUTPUT_TOPIC = "sb_gps_output"; // this node's output: pub
double lat, lon, goal_lat, goal_lon, dist, x_dist, y_dist;
struct waypoint current_waypoint;
struct waypoint goal_waypoint;
double* waypoints_list = ReturnWaypoints();
int size = sizeof(waypoints_list)*2, c = 0;


int main(int argc, char **argv) 
{

	// hard code current waypoint
	current_waypoint.long_x = 24;
	current_waypoint.lat_y = 100;

	ros::init(argc, argv, GPS_NODE_NAME); // start node
	ros::NodeHandle nh; // main access point to communication with ROS system. First one initializes node.

	ros::Publisher gps_data_pub = nh.advertise<geometry_msgs::Twist>(GPS_OUTPUT_TOPIC, 1000); // publisher for output

	ros::Rate loop_rate(10); // 10hz loop rate

	int next_waypoint = GetWaypoint();

	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;

	while (ros::ok()) // false when Ctrl-C, kicked off network, ros::shutdown(), all Nodehandles destroyed.
	{
		CalculateDistance();
		if (x_dist / abs(x_dist) == -1) { msg.angular.z = -1; }
		else { msg.angular.z = 1; }
		msg.linear.x = x_dist;
		msg.linear.y = y_dist;
		gps_data_pub.publish(msg);
		ROS_INFO("Linear.x = %f\nLinear.y = %f\nAngular.z = %f\n", msg.linear.x, msg.linear.y, msg.angular.z);
		ros::spinOnce();
		loop_rate.sleep(); // sleep for 10hz
		next_waypoint = GetWaypoint();
	}
	return 0;
}


// Get the next waypoint: returns n for the nth waypoint, starting at 1
// return -1 for 'no more waypoints'
int GetWaypoint() 
{
	if (c >= size ) { c = 0; }
	goal_waypoint.long_x = waypoints_list[c];
	goal_waypoint.lat_y = waypoints_list[c+1];
	c += 2;
	return (c+1)/2;
}


// Calculates Euclidean distance from position to goal
void CalculateDistance()
{
	x_dist = goal_waypoint.long_x - current_waypoint.long_x;
	y_dist = goal_waypoint.lat_y - current_waypoint.lat_y;
	dist = sqrt(pow(x_dist, 2.0) + pow(y_dist, 2.0));
}	


double* ReturnWaypoints() 
{
	string output;
	int count = 0;
	int array_size;
	double* waypoints_array = NULL;
	ifstream waypoints_file ("/home/jechli/snowbots_ws/src/WaypointsTxt/waypoints.txt");
	if (waypoints_file.is_open())
	{
		while (getline(waypoints_file, output))
		{
			if (count == 0) {
				array_size = atoi(output.c_str())*2;
				waypoints_array = new double [array_size];
				cout << array_size << "\n";
				count++;
			}
			
			else { 
				waypoints_array[count-1] = atof(output.c_str());
				count++;
				cout << waypoints_array[count-1] << "\n"; 
			}
		}
		waypoints_file.close();
	}

	else
	{
		cout << "Unable to open file";
	}

	return waypoints_array;
}	
    

