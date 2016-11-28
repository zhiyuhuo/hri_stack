#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include "nav_msgs/Odometry.h"

#include "../Universal/Header.h"

#include "Header.h"

using namespace std;

void Robot::poseCallback(const nav_msgs::OdometryConstPtr& msg)
{
	float x, y, qx, qy, qz, qw, theta;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;
	float rotz = atan2(2*qw*qz, 1-2*qz*qz);
	theta = rotz;
	
	while (theta < 0)
	{
		theta += 2*PI;
	}
	
	while (theta > 2*PI)
	{
		theta -= 2*PI;
	}

	m_posRobot.SetX(x);
	m_posRobot.SetY(y);
	m_theta = theta;	
	
	m_ifGetPose = true;
// 	cout << "x: " << m_posRobot.GetX() << ",	y: " << m_posRobot.GetY() << ",		theta: " << m_theta << endl;
}

int Robot::SetOccupancyMap(int width, int height, double resolution, double originX, double originY)
{
        cout << "set grid map" << endl;
	fill(m_occupiedMap.begin(), m_occupiedMap.end(), 0);
	m_occupiedMap.resize(width*height, 0);
  
	for (int n = 0; n < m_entities.size(); n++)
	{
		if (m_entities[n].name == "CR" || m_entities[n].name == "OR")
		{
		    continue;
		}
		
		Ent ent = m_entities[n];
		int u, v;
		for (int i = 0; i < ent.vec.size() / 2; i++)
		{
			u = (int)((ent.vec[2*i] - originX) / resolution);
			v = (int)((ent.vec[2*i+1] - originY) / resolution);
			m_occupiedMap[u + v * width] = 255;
		}
	}
	/*
	nav_msgs::SetMap srvSetMap;
	srvSetMap.request.map.info.resolution = resolution; 
	srvSetMap.request.map.info.width = width; 
	srvSetMap.request.map.info.height = height; 
	srvSetMap.request.map.info.origin.position.x = originX;
	srvSetMap.request.map.info.origin.position.y = originY;
		
	for (int i = 0; i < m_occupiedMap.size(); i++)
	{
		srvSetMap.request.map.data.push_back(m_occupiedMap[i]);
	}
	srvSetMap.request.initial_pose.pose.pose.position.x = m_posRobot.GetX();
	srvSetMap.request.initial_pose.pose.pose.position.y = m_posRobot.GetY();
		
	if (m_setMapClient.call(srvSetMap))
	{
		ROS_INFO("Res: %d", (int)srvSetMap.response.success);
	}
	else
	{
		ROS_ERROR("Failed to call set map");
		return -1;
	}
	*/
	return 1;
}

vector<VecPosition> Robot::CallForPathPlan(VecPosition posStart, VecPosition posTarget)
{
	nav_msgs::GetPlan srvGetPlan;
	srvGetPlan.request.start.pose.position.x = posStart.GetX(); 
	srvGetPlan.request.start.pose.position.y = posStart.GetY(); 
	srvGetPlan.request.goal.pose.position.x = posTarget.GetX(); 
	srvGetPlan.request.goal.pose.position.y = posTarget.GetY(); 
	
	vector<VecPosition> res;
	if (m_getPlanClient.call(srvGetPlan))
	{
		for (int i = 0; i < srvGetPlan.response.plan.poses.size(); i++)
		{
			VecPosition step(srvGetPlan.response.plan.poses[i].pose.position.x,
					srvGetPlan.response.plan.poses[i].pose.position.y);
			res.push_back(step);
			cout << step.GetX() << " " << step.GetY() << endl;
		}
	}
	else
	{
		ROS_ERROR("Failed to call service get path");
	}		
	
	return res;
}

#endif