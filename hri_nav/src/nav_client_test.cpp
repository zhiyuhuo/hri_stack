#include "ros/ros.h"
#include <cstdlib>
#include <stdio.h>
#include <vector>
#include "nav_msgs/SetMap.h"
#include "nav_msgs/GetPlan.h"

#include "hri_nav/Universal/Header.h"
#include "hri_nav/Planner/Header.h"
#include "hri_nav/Geometry/Header.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nav_client");
	ros::NodeHandle n;
	
	//SetMap Service
	ros::ServiceClient clientSetMap = n.serviceClient<nav_msgs::SetMap>("nav_set_map");
	nav_msgs::SetMap srvSetMap;
	srvSetMap.request.map.info.resolution = 0.25; 
	srvSetMap.request.map.info.width = 400; 
	srvSetMap.request.map.info.height = 400; 
	srvSetMap.request.map.info.origin.position.x = -50;
	srvSetMap.request.map.info.origin.position.y = -50;
	
	vector<uint8_t> data(srvSetMap.request.map.info.width * srvSetMap.request.map.info.height, 0);
	for (int i = 0; i < data.size(); i++)
	{
		srvSetMap.request.map.data.push_back(data[i]);
	}
	srvSetMap.request.initial_pose.pose.pose.position.x = 0;
	srvSetMap.request.initial_pose.pose.pose.position.y = 0;
	
	if (clientSetMap.call(srvSetMap))
	{
		ROS_INFO("Res: %d", (int)srvSetMap.response.success);
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
	
	//GetPlan Service
	//SetMap Service
	ros::ServiceClient clientGetPlan = n.serviceClient<nav_msgs::GetPlan>("nav_get_plan");
	nav_msgs::GetPlan srvGetPlan;
	srvGetPlan.request.start.pose.position.x = 1.0; 
	srvGetPlan.request.start.pose.position.y = 1.0; 
	srvGetPlan.request.goal.pose.position.x = 3.84; 
	srvGetPlan.request.goal.pose.position.y = 2.73; 
	
	vector<VecPosition> steps;
	if (clientGetPlan.call(srvGetPlan))
	{
		for (int i = 0; i < srvGetPlan.response.plan.poses.size(); i++)
		{
			VecPosition step(srvGetPlan.response.plan.poses[i].pose.position.x,
					srvGetPlan.response.plan.poses[i].pose.position.y);
			steps.push_back(step);
			cout << step.GetX() << " " << step.GetY() << endl;
		}
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}	

	return 0;
}