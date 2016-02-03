#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include <vector>

#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <pthread.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "hri_pbd5/Universal/Header.h"
#include "hri_pbd5/Geometry/Header.h"
#include "hri_pbd5/Robot/Header.h"

//#define SPATIALCOMMAND

using namespace std;
using namespace cv;

float posX, posY, posTheta;
vector<vector<float> > laserData;
geometry_msgs::Twist speedMsg;

Robot _robot;
string newComeVoiceMsg;
string spatialCommandStr;
int ifNewCommand = 0;
int ifTerminated = 0;

void ListenCallbackLaser(const sensor_msgs::LaserScan& msg);
void ListenCallbackPose(const nav_msgs::OdometryConstPtr& msg);
void ListenCallbackCMD(const std_msgs::StringPtr& msg);

bool ifGetPose = false;
bool ifGetLaser = false;
float hx = 0;
float hy = 0;
float ht = 0;

int main(int argc, char **argv)
{
	printf("");
	ros::init(argc, argv, "read_detector");
	ros::NodeHandle n;
	
	//Robot Staff
	ros::Subscriber laserSub = n.subscribe("base_scan", 1000, ListenCallbackLaser);
	ros::Subscriber poseSub = n.subscribe("base_pose_ground_truth", 1000, ListenCallbackPose);
	ros::Subscriber cmdSub = n.subscribe("human_cmd", 1000, ListenCallbackCMD);
	
	//Odometry
	posX = 0;
	posY = 0;
	posTheta = PI / 2;
	
	//Motor Pub
	ros::Publisher speedPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	cout << "robot loop start..." << endl;
	while (ros::ok() && ifTerminated == 0)
	{	
		_robot.m_laserData = laserData;
		_robot.m_laserUnitNumber = laserData.size();
		_robot.m_posRobot.SetX(posX);
		_robot.m_posRobot.SetY(posY);
		_robot.m_theta = posTheta;	
		
////Robot Main Loop
		_robot.TestRead();
		exit(1);
////Robot Main Loop End
	
		speedMsg.linear.x = _robot.m_linearSpeed;
		speedMsg.angular.z = _robot.m_angularSpeed;
		speedPub.publish(speedMsg);		
		
		ros::spinOnce();
	}
	
	if (newComeVoiceMsg.compare("terminate") == 0)
	{
		speedMsg.linear.x = 0;
		speedMsg.angular.z = 0;
		speedPub.publish(speedMsg);	
	}
	  
	return 0;
}

void ListenCallbackLaser(const sensor_msgs::LaserScan& msg)
{
	laserData.clear();
	vector<float> ranges = msg.ranges;
	vector<float> intensities = msg.intensities;
	int N = ranges.size();
	float minAngle = msg.angle_min;
	float angleIncrement = msg.angle_increment;
	for (int i = 0; i < N; i++)
	{
		vector<float> laserunit;
		laserunit.push_back(minAngle + i * angleIncrement);
		laserunit.push_back(ranges[i]);
		laserunit.push_back(intensities[i]);
		laserData.push_back(laserunit);
	}
	ifGetLaser = true;
}

void ListenCallbackPose(const nav_msgs::OdometryConstPtr& msg)
{
	float x, y, qx, qy, qz, qw, th, theta;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;
	float rotz = atan2(2*qw*qz, 1-2*qz*qz);
	th = rotz;

	theta = th;
	if (th < 0)
	{
		theta = th + 2 * PI;
	}

	posX = x;
	posY = y;
	posTheta = theta;
	ifGetPose = true;
}

void ListenCallbackCMD(const std_msgs::StringPtr& msg)
{
	string cmd = msg->data;
	_robot.m_humanCmd = cmd;
}


