/*
 * Commander Node
 * Intelligent Ground Vehicle Challenge 2014
 * Oakland University - Rochester, Michigan
 * 
 * Team UBC Snowbots
 * University of British Columbia - Vancouver, BC
 *
 */

#include <iostream>
#include <stdlib.h>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

#include "sb_msgs/CarCommand.h"
#include "sb_msgs/LidarNav.h"

// Global Constants
static const int SECOND = 1000000; //1 million us

// steering = 1 is left in stage so steering = -1 is right
geometry_msgs::Twist twistConvertor(sb_msgs::CarCommand car_msg);

using namespace ros;
using namespace std;

struct NavCommand
{
    NavCommand() : throttle(0), steering(0) {}
    double throttle;
    double steering;
};

// ROS-related Constants
static const string NODE_NAME               = "commander";
static const string CAR_PUBLISH_TOPIC       = "cmd_vel";
static const string LIDAR_SUBSCRIBE_TOPIC   = "lidar_nav";  // lidar_nav node suggestion
static const string VISION_SUBSCRIBE_TOPIC  = "vision_nav"; // vision_nav node suggestion
static const string GPS_SUBSCRIBE_TOPIC     = "gps_nav";    // gps_nav node
static const int LOOP_FREQ = 30; // Hz

NavCommand lidar_command;
NavCommand vision_command;
NavCommand gps_command;

double throttle = 0;
double steering = 0; // 1 is right, 30 shown on arduino driver (real world), -1 is left

bool stopSignFlag = false;
bool redLightFlag = false;

void Stop()
{
	throttle = 0;
	steering = 0;		
}

//callback for vision directions
void vision_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  vision_command.throttle = msg->linear.y;
  vision_command.steering = msg->angular.z;
}


//callback for lidar directions
void lidar_state_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  lidar_command.throttle = msg->linear.y;
  lidar_command.steering = msg->angular.z;   
}

//callback for gps directions
void gps_callback(const geometry_msgs::Twist::ConstPtr& msg) // should we change to carcommand? The twist msg here outputs into a topic called "vision_vel"
{
	ROS_INFO("\nGot back: [Lin.x = %f, Lin.y = %f, Ang.z = %f]", msg->linear.x, msg->linear.y, msg->angular.z);
	gps_command.throttle = msg->linear.y;
  	gps_command.steering = msg->angular.z;
}


geometry_msgs::Twist driver()
{
    geometry_msgs::Twist car_msg;

        
    
    //uncomment the two lines below and populate car_msg with data 
    car_msg.linear.y  = throttle;	//Throtle;
    car_msg.angular.z = steering;	//Steering;
     
    // geometry_msgs::Twist twist_msg = twistConvertor(car_msg);

    return car_msg;
}


int main( int argc, char** argv )
{
 
    //ros initialization
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    
    //subscribes to IR topic to receive data from arduino
    //lidar_class my_lidar_class;

    ros::Subscriber LIDAR_State = n.subscribe(LIDAR_SUBSCRIBE_TOPIC, 20, lidar_state_callback);
    ros::Subscriber vision_State = n.subscribe(VISION_SUBSCRIBE_TOPIC, 20, vision_callback);
    ros::Subscriber GPS_State = n.subscribe(GPS_SUBSCRIBE_TOPIC, 20, gps_callback);

    ros::Publisher car_pub = n.advertise<geometry_msgs::Twist>(CAR_PUBLISH_TOPIC, 1);

    //controls how fast it goes
    ros::Rate loop_rate(LOOP_FREQ);

    ROS_INFO("ready to go");
    
    //give it 3 seconds before it starts moving!
    usleep(3*SECOND);
       
    ROS_INFO("going");   
    
    while(ros::ok())
    {
	
        //driver is navigation function
    	
        //publshing data to robot
       // ROS_INFO("sending throttle=%f, steering=%f", twist_msg.linear.x, twist_msg.angular.z);

	geometry_msgs::Twist car_msg = driver();
        car_pub.publish(car_msg);
        
        //checking callbacks and sleeping to match loop rate
        ros::spinOnce();
        loop_rate.sleep();	
    }
    ROS_INFO("shutting down node");
  
    return 0;
}

/*
geometry_msgs::Twist twistConvertor(sb_msgs::CarCommand car_msg)
{
      //Creating instances of Twist and Vector3 classes	
      geometry_msgs::Twist twistReturn;
      geometry_msgs::Vector3 linearVel;
      geometry_msgs::Vector3 angularVel;

      //Since the robot can move in only one direction (x-axis),
      //we set the other two to zero	
      linearVel.x = car_msg.throttle;
      linearVel.y = 0;
      linearVel.z = 0;

      //Since the robot can turn around z-axis only,
      //we set the other two to zero
      angularVel.x = 0;
      angularVel.y = 0;
      angularVel.z = car_msg.steering;

      twistReturn.linear = linearVel;
      twistReturn.angular = angularVel;

      return twistReturn;
}
*/
