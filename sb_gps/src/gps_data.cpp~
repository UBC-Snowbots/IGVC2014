#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h> // vel
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/String.h>
#include <std_msgs/Vector3.h> // is this right?
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>

static const string GPS_NODE_NAME = "gps_node";
static const string GPS_POSITION_TOPIC = "fix"; // gps: sub
static const string GPS_TIMESTAMP_TOPIC = "time_reference"; // timestamp of original gps reading: sub
static const string GPS_VELOCITY_TOPIC = "vel"; // velocity reading by gps 
static const string GPS_OUTPUT_TOPIC = "sb_gps_output"; // this node's output: pub
double lat, lon, goal_lat, goal_lon, dist, x_dist, y_dist;


// for subscribers
void positionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	lat = msg->latitude; // float64
	lon = msg->longitude; // float64
	//msg->status
	//msg->altitude // float64
	// msg->covariance?
}

void timeStampCallback(const sensor_msgs::TimeReference::ConstPtr& msg)
{
	ROS_INFO("Timestamp: %s", msg->time_ref->data.c_str()); //time
}

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
{
	//msg->twist->linear // vector3
	//msg->twist->angular // vector3
}

void calculateDistance(const double x, const double y, const double goal_x, const double goal_y)
{
	x_dist = goal_x - x;
	y_dist = goal_y - y;
	dist = sqrt(pow(x_dist, 2.0) + pow(y_dist, 2.0));
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, GPS_NODE_NAME); // start node
	ros::NodeHandle nh; // main access point to communication with ROS system. First one initializes node.
	ros::Publisher gps_data_pub = nh.advertise<SOME SB_GPS_MSG>(GPS_OUTPUT_TOPIC, 1000); // publisher for output
	ros::Subscriber position_sub = nh.subscribe(GPS_POSITION_TOPIC, 1000, positionCallback);
	ros::Subscriber timestamp_sub = nh.subscribe(GPS_TIMESTAMP_TOPIC, 1000, timeStampCallback);
	ros::Subscriber velocity_sub = nh.subscribe(GPS_VELOCITY_TOPIC, 1000, velocityCallback);
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

		
    

