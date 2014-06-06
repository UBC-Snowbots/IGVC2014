/*
 * lidar_force_nav
 * Intelligent Ground Vehicle Challenge 2014
 * Oakland University - Rochester, Michigan
 * 
 * UBC Snowbots
 * June 2014
 */


#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sb_msgs/CarCommand.h>

using namespace ros;
using namespace std;

//Global constatns
static const double IGNORE_ANGLE = 3.1415265; //pi radians = 90 degrees
static const int    OFFSET_ANGLE = 20;        // offset from central ray
static const double REDZONE      = 1.0;
static const double ORANGEZONE   = 2.0;
static const double SLOW_SPEED	 = 0.2;

//ros related constants
static const string NODE_NAME       = "lidar_nav";
static const string SUBSCRIBE_TOPIC = "scan";
static const string PUBLISH_TOPIC   = "commander";
static int LOOP_FREQ = 30;
geometry_msgs::Vector3 directions;
sb_msgs::CarCommand car_command;

double clamp (double in, double cap)
{
	if      ( in >  cap) return cap;
	else if ( in < -1 * cap) return (-1 * cap);
	else                 return in;	
}

//call back function
void callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{
	int num_rays = msg_ptr->ranges.size();
	double x_total = 0.0;
	double y_total = 0.0;
	int valid_rays = 0;

	for(int i =0; i < num_rays;i++)
	{
		float angle = msg_ptr->angle_min + i*(msg_ptr->angle_increment);
		float dist = msg_ptr->ranges[i];

		if(angle < -IGNORE_ANGLE || angle > IGNORE_ANGLE)
			continue;
		if(dist > msg_ptr->range_max)
			continue;

		if (dist != 0 && isinf(dist) == 0 && isnan(dist) == 0)
		{
			float force = -1.0/dist;
			if (isnan(cos(angle)) == false && isnan(sin(angle)) == false)
			{
				x_total += force * cos(angle);
				y_total += force * sin(angle);
				valid_rays++;
			}
		}		
	}
	if(valid_rays <= 0)
	{
		ROS_FATAL("No valid rays found");
		return;
	}

	if (valid_rays != 0)
	{
	//	directions.x = -1 * x_total / valid_rays;
	//	directions.y = 1  * y_total / valid_rays;
	//	directions.z = 0;
	//	directions.x = clamp(directions.x);
	//	directions.y = clamp(directions.y);
		car_command.throttle = -1 * x_total / valid_rays;
		car_command.steering =  1 * y_total / valid_rays;
		car_command.priority = 0.5;
		car_command.throttle = clamp(car_command.throttle, 0.5);
		car_command.steering = clamp(car_command.steering, 1);
	//	printf("x_total: %f   y_total: %f\n", x_total, y_total);
	}

	// check for blockages
	for(int i =num_rays-OFFSET_ANGLE; i < num_rays+OFFSET_ANGLE;i++)
	{
		float angle = msg_ptr->angle_min + i*(msg_ptr->angle_increment);
		float dist = msg_ptr->ranges[i];

		if (dist < REDZONE)
		{
			// stop moving forward
			car_command.throttle = 0;
			car_command.priority = 1;		
		}
		else if (dist < ORANGEZONE)
		{
			car_command.throttle = SLOW_SPEED;
			car_command.priority = 0.8;	
		}
	}
}

geometry_msgs::Twist twist_converter(sb_msgs::CarCommand cc)
{
	geometry_msgs::Twist twist;
	geometry_msgs::Vector3 Linear;
	geometry_msgs::Vector3 Angular;
	
	
	twist.linear.x = 0;
	twist.linear.y = cc.throttle;
	twist.linear.z = 0;

	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = cc.steering;

	return twist;	
}


int main (int argc, char** argv)
{
	init(argc, argv,NODE_NAME);
	NodeHandle n;
	

	Subscriber lidar_state = n.subscribe(SUBSCRIBE_TOPIC,20,callback);
	
	Publisher car_pub = n.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC,1);
	Rate loop_rate(LOOP_FREQ);
	ROS_INFO("ready to go");
	
	ROS_INFO("going");
	while(ros::ok())
	{
	//	geometry_msgs::Twist twistMsg = twist_converter(car_command);
	//	ROS_INFO("the twist vector is %f , %f", twistMsg.linear.y, twistMsg.angular.z);
		ROS_INFO("CarCommand - throttle: %f , steering: %f , priority: %f", car_command.throttle, car_command.steering, car_command.priority); 
		car_pub.publish(car_command);
	  	ros::spinOnce();
    		loop_rate.sleep();	
  	}
  ROS_INFO("shutting down node");
  
  return 0;
}
