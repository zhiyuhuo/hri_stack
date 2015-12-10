#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include "nav_msgs/Odometry.h"

#include "../Universal/Header.h"

#include "Header.h"

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
	
	cout << "x: " << m_posRobot.GetX() << ",	y: " << m_posRobot.GetY() << ",		theta: " << m_theta << endl;
}

#endif