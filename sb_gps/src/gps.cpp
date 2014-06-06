#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "sb_msgs/CarCommand.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

// REMEMBER TO EDIT WAYPOINTS FILE DIRECTORY TODO
// need to add in compass direction still

using namespace std;

struct waypoint 
{
	double long_x;
	double lat_y;
};

// functions
int GetWaypoint();
void CalculateAngle(); 
double* ReturnWaypoints();
void CheckWaypointStatus();

// variables
static const string GPS_NODE_NAME = "gps_node"; 
static const string GPS_OUTPUT_TOPIC = "gps_nav"; 
static const string GPS_TEST_TOPIC = "vision_vel"; // test sub
static const string GPS_INPUT_TOPIC = "gps_state"; 
bool isAtGoal = false, isFinished = false; 
double lat, lon, goal_lat, goal_lon, angle, x_dist, y_dist, current_direction;
struct waypoint current_waypoint;
struct waypoint goal_waypoint;
double* waypoints_list = ReturnWaypoints();
int size = sizeof(waypoints_list), c = 0, lat_exp, long_exp;



// temp callback
void gpsCallback(const std_msgs::String::ConstPtr& msg) // changed to string
{
	sscanf(msg->data.c_str(), "B,%lfe+%d,%lfe+%d,%lf", &current_waypoint.lat_y, &lat_exp, &current_waypoint.long_x, &long_exp, &current_direction);
	lat_exp -= 7; lat_exp = pow(10, lat_exp);
	long_exp -= 7; long_exp = pow(10, long_exp);
	current_waypoint.lat_y *= lat_exp;
	current_waypoint.long_x *= long_exp;
}


int main(int argc, char **argv) 
{

	// hard code current waypoint 
	current_waypoint.long_x = 0; // keep like this until we start receiving gps data
	current_waypoint.lat_y = 0;
	current_direction = 0;

	ros::init(argc, argv, GPS_NODE_NAME); // start node
	ros::NodeHandle nh; // main access point to communication with ROS system. First one initializes node.

	ros::Publisher gps_test_pub = nh.advertise<geometry_msgs::Twist>(GPS_TEST_TOPIC, 20); 
	ros::Publisher gps_data_pub = nh.advertise<sb_msgs::CarCommand>(GPS_OUTPUT_TOPIC, 20);
	ros::Subscriber gps_sub = nh.subscribe(GPS_INPUT_TOPIC, 20, gpsCallback); 

	ros::Rate loop_rate(5); // 10hz loop rate

	int next_waypoint = GetWaypoint();

	geometry_msgs::Twist t_msg;
	// do nothing for linear
	t_msg.linear.x = 1;
	t_msg.linear.y = 1;
	t_msg.linear.z = 0;
	t_msg.angular.x = 0;
	t_msg.angular.y = 0;
	// angular.z is angle in ratio (1 <= theta <= -1)
	t_msg.angular.z = 0;


	sb_msgs::CarCommand cc_msg;
	cc_msg.throttle = 1;
	cc_msg.steering = 0;
	cc_msg.priority = 1;

	while (ros::ok()) 
	{
		CalculateAngle(); // calculate angular.z
		t_msg.angular.z = angle;
		cc_msg.steering = angle;
		gps_test_pub.publish(t_msg);
		gps_data_pub.publish(cc_msg);

		//ROS_INFO("\nLinear.x = %f\nLinear.y = %f\nAngular.z = %f\n", msg.linear.x, msg.linear.y, msg.angular.z);

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
	// estimated to within ~11cm
	int cx, cy, gx, gy;
	cx = ceil(current_waypoint.long_x*100000)/100000;
	cy = ceil(current_waypoint.lat_y*100000)/100000;
	gx = ceil(goal_waypoint.long_x*100000)/100000;
	gy = ceil(goal_waypoint.lat_y*100000)/100000;
	if (cx == gx && cy == gy) { isAtGoal = true; }

}


// Calculates Euclidean distance from position to goal
void CalculateAngle()
{
	x_dist = goal_waypoint.long_x - current_waypoint.long_x;
	y_dist = goal_waypoint.lat_y - current_waypoint.lat_y;

	if (current_waypoint.long_x == 0 || current_waypoint.lat_y == 0) { angle = 0; return; } 
	else if (current_waypoint.long_x == goal_waypoint.long_x) {
		if (y_dist / abs(y_dist) == -1) { angle = 180.00; }
		else { angle = 0.00; }
	}
	else if (current_waypoint.lat_y == goal_waypoint.lat_y) {
		if (x_dist / abs(x_dist) == -1) { angle = 270.00; }
		else { angle = 90.00; }
	}
	else {
		angle = atan(abs(x_dist)/abs(y_dist)); 
		if (x_dist / abs(x_dist) == -1) {
			if (y_dist / abs(y_dist) == -1) { angle += 180.00; }
			else { angle += 270.00; } 		
		}
		else { // if x positive
			if (y_dist / abs(y_dist) == -1) { angle += 90.0; }
		}
	}

	if (abs(current_direction - angle) <= 5.00) { angle = 0; return; }
	else if (abs(current_direction - angle) < 180.00) { angle = current_direction - angle; }
	else { angle = 360.00 - current_direction + angle; angle *= -1;}  
	angle = angle / 180.00;
	return;
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
    

