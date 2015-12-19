#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <pthread.h>

#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "p2os_msgs/MotorState.h"
#include "p2os_msgs/SonarArray.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf2_msgs/TFMessage.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define GAZEBO

// #define IMG_WIDTH 640
// #define IMG_HEIGHT 480
// #define IMG_SIZE 640*480
// #define IMG_SIZE3 640*480*3
// #define PI 3.1416

#include "hri_nav/Universal/Header.h"
#include "hri_nav/Planner/Header.h"
#include "hri_nav/Geometry/Header.h"

using namespace std;

//sensors
float posX, posY, posTheta;
geometry_msgs::Twist speedMsg;

void ListenCallbackPose(const nav_msgs::OdometryConstPtr& msg);


int main(int argc, char **argv)
{
	// initialization global variables
	posX = 0;
	posY = 0;
	posTheta = 0;
  
	//printf("");
	ros::init(argc, argv, "hri_nav");
	ros::NodeHandle n;
	
	// subscribed topics

	#ifdef GAZEBO
	ros::Subscriber poseSub = n.subscribe("hri_robot/odom", 1000, ListenCallbackPose);
	#else
	ros::Subscriber poseSub = n.subscribe("/pose", 1000, ListenCallbackPose);
	#endif
	
	#ifdef GAZEBO
	ros::Publisher speedPub = n.advertise<geometry_msgs::Twist>("hri_robot/cmd_vel", 10);
	#else
	ros::Publisher motorstatePub = n.advertise<p2os_msgs::MotorState>("/cmd_motor_state", 10);
	ros::Publisher speedPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	#endif
	
	// Define messages
	p2os_msgs::MotorState motorMsg;
	geometry_msgs::Twist speedMsg;
	
	// PreProcessing
	printf("start robot...\n");
	for (int i = 0; i < 5; i++) // preprocess to fix the tilt
	{
		struct timespec timeOut,remains;
		timeOut.tv_sec = 0;
		timeOut.tv_nsec = 500000000; // 50 milliseconds 
		nanosleep(&timeOut, &remains);
		printf("%d\n",4-i);	
		motorMsg.state = (int)1;
		
		#ifndef GAZEBO
		motorstatePub.publish(motorMsg);
		#endif
	}
	
	
	Planner planner;
	planner.PathPlanning();
	exit(1);
	
		
	ros::Rate loopRate(30);
	cout << "robot loop start..." << endl;
	while (ros::ok())
	{	
		
		
		speedPub.publish(speedMsg);		
		
		ros::spinOnce();
		loopRate.sleep();
	}
	  
	return 0;
}

void ListenCallbackPose(const nav_msgs::OdometryConstPtr& msg)
{
	float x, y, qx, qy, qz, qw, th;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;
	float rotz = atan2(2*qw*qz, 1-2*qz*qz);
	th = rotz;

	while (th < 0)
	{
		th += 2*PI;
	}
	
	while (th > 2*PI)
	{
		th -= 2*PI;
	}
	
	//cout << x << ", " << y << ", " << th << endl;

	posX = x;
	posY = y;
	posTheta = th;
}

