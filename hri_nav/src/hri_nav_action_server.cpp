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

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <p2os_msgs/MotorState.h>
#include <p2os_msgs/SonarArray.h>
#include <nav_msgs/SetMap.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

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

Planner _planner;

//planner
bool SetMapCallback(nav_msgs::SetMap::Request  &req, nav_msgs::SetMap::Response &res);
bool GetPlanCallback(nav_msgs::GetPlan::Request  &req, nav_msgs::GetPlan::Response &res);


int main(int argc, char **argv)
{ 
	//printf("");
	ros::init(argc, argv, "hri_nav");
	ros::NodeHandle n;
	
	cout << "start nav service..." << endl;	
	
	ros::ServiceServer serviceSetMap = n.advertiseService("nav_set_map", SetMapCallback);
	ros::ServiceServer serviceGetPlan = n.advertiseService("nav_get_plan", GetPlanCallback);
	
	ros::spin();
	  
	return 0;
}

bool SetMapCallback(nav_msgs::SetMap::Request  &req, nav_msgs::SetMap::Response &res)
{
	double mapResolution = (double)req.map.info.resolution;
	long mapWidth = req.map.info.width;
	long mapHeight = req.map.info.height;
	geometry_msgs::Pose mapOrigin = req.map.info.origin;
	vector<uint8_t> mapData(mapWidth*mapHeight);
	for (int i = 0; i < mapWidth*mapHeight; i++)
	{
		mapData[i] = req.map.data[i];
	}
	
	VecPosition origin(mapOrigin.position.x, mapOrigin.position.y);
	_planner.SetMap(mapWidth, mapHeight, mapResolution, origin, mapData);
	
	geometry_msgs::Pose initPose = req.initial_pose.pose.pose;
	
	res.success = true;
	
	return true;
}


bool GetPlanCallback(nav_msgs::GetPlan::Request  &req, nav_msgs::GetPlan::Response &res)
{
	VecPosition posStart(req.start.pose.position.x, req.start.pose.position.y);
	VecPosition posTarget(req.goal.pose.position.x, req.goal.pose.position.y);
	
	vector<VecPosition> stepsVecP = _planner.GetPlan(posStart, posTarget);
	
	
	for (int i = 0; i < stepsVecP.size(); i++)
	{
		geometry_msgs::PoseStamped posestamped;
		posestamped.pose.position.x = stepsVecP[i].GetX();
		posestamped.pose.position.y = stepsVecP[i].GetY();
		res.plan.poses.push_back(posestamped);
	}
	
	return true;
}

