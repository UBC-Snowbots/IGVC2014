#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include "read_waypoints.h"

using namespace std;

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

	ros::Publisher gps_data_pub = nh.advertise<std_msgs::String>(GPS_OUTPUT_TOPIC, 1000); // publisher for output

	ros::Rate loop_rate(10); // 10hz loop rate
  
	int next_waypoint = GetWaypoint();
	while (ros::ok()) // false when Ctrl-C, kicked off network, ros::shutdown(), all Nodehandles destroyed.
	{
		if (next_waypoint == -1) { 
			next_waypoint = GetWaypoint(); 
		} // keep looping
		// broadcast data TODO
		std_msgs::String msg;
		stringstream ss;
		CalculateDistance();
		ss << dist; // this is the message we need to publish
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str()); // cout
		gps_data_pub.publish(msg);
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
	//else {
		goal_waypoint.long_x = waypoints_list[c];
		goal_waypoint.lat_y = waypoints_list[c+1];
		c += 2;
	//}
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
	ifstream waypoints_file ("/home/jechli/Documents/Snowbots/IGVC2013/sb_gps/WaypointsTxt/waypoints.txt");
	if (waypoints_file.is_open())
	{
		while (getline(waypoints_file, output))
		{
			if (count == 0) {
				array_size = atoi(output.c_str())*2;
				waypoints_array = new double [array_size];
				//cout << array_size << "\n";
				count++;
			}
			
			else { 
				waypoints_array[count-1] = atof(output.c_str());
				count++;
				//cout << waypoints_array[count-1] << "\n"; 
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
    

