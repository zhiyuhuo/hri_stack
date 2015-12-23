#ifndef ACTION_H
#define ACTION_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>

#include "geometry_msgs/Twist.h"
#include "Header.h"

int Robot::ToAngle(float angleTarget)
{
	double angleRobot = m_theta;
	double deltaAngle = angleTarget - angleRobot;
	while(deltaAngle > PI)
	{
		deltaAngle -= 2 * PI;
	}
	while(deltaAngle < -PI)
	{
		deltaAngle += 2 * PI;
	}

	double kA = abs(deltaAngle / PI) * 0.02;
	if (kA < 0.5)
	{
		kA = 1.0;
	}
	m_linearSpeed = 0;
	m_angularSpeed = deltaAngle * kA;

	if (abs(deltaAngle) < PI/100)
	{
		m_linearSpeed = 0;
		m_angularSpeed = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

int Robot::ToPos(VecPosition posTarget)
{
	VecPosition posRobot = m_posRobot;
	double angleTarget = posTarget.GetDirectionTo(posRobot);
	double angleRobot = m_theta;
	double deltaDist = (posTarget - posRobot).GetMagnitude();
	double deltaAngle = angleTarget - angleRobot;
	while(deltaAngle > PI)
	{
		deltaAngle -= 2 * PI;
	}
	while(deltaAngle < -PI)
	{
		deltaAngle += 2 * PI;
	}
	
	double kL, kAC, kAL, kA;
	kL = 0.03;
	kAC = 5;
	kAL = 1.0;
	
	kA = abs(deltaAngle / PI) * kAC;
	if (kA < kAL)
	{
		kA = kAL;
	}
	m_linearSpeed = (PI - abs(deltaAngle)) * kL;
	m_angularSpeed = deltaAngle * kA;

	/*
	if (abs(deltaAngle) < PI / 2)
	{
		kA = abs(deltaAngle / PI) * kAC;
		if (kA < kAL)
		{
			kA = kAL;
		}
		m_linearSpeed = (PI - abs(deltaAngle)) * kL;
		m_angularSpeed = deltaAngle * kA;
	}
	else if (deltaAngle > PI / 2)
	{
		deltaAngle = PI - deltaAngle;
		kA = abs(deltaAngle / PI) * kAC;
		if (kA < kAL)
		{
			kA = kAL;
		}
		m_linearSpeed =  - (PI - abs(deltaAngle)) * kL;
		m_angularSpeed = - deltaAngle * kA;		
	}
	else if (deltaAngle < -PI / 2)
	{
		deltaAngle = -PI - deltaAngle;
		kA = abs(deltaAngle / PI) * kAC;
		if (kA < kAL)
		{
			kA = kAL;
		}
		m_linearSpeed =  - (PI - abs(deltaAngle)) * kL;
		m_angularSpeed = - deltaAngle * kA;		
	}*/

	if (deltaDist < 0.1)
	{
		m_linearSpeed = 0;
		m_angularSpeed = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

int Robot::ToPosAngle(VecPosition posTarget, float angleTarget)
{
	int r1, r2 = 0;
	r1 = ToPos(posTarget);
	//r1 = ToPositionAvoidObstacles(posTarget);
	if (r1 > 0)
	{
		if (angleTarget >= 0)
		{
			r2 = ToAngle(angleTarget);
		}
		else
		{
			r2 = 1;
		}
	}
	if (r1 > 0 && r2 > 0)
	{
		return 1;
	}
	return 0;
}

int Robot::ToPositionAvoidObstacles(VecPosition posTarget)
{
	int res = 0;
	VecPosition posRobot = m_posRobot;
	double deltaDist = (posTarget - posRobot).GetMagnitude();
	float D = 0.50;
	
	float obs[12] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	
	for (float x = -1.0; x < 1.0; x += 0.1)
	{
		for (float y = -1.0; y < 1.0; y += 0.1)
		{
			vector<int> uv = CoordinateToPixel(x + posRobot.GetX(), y + posRobot.GetY());
			int s = uv[0] + 100 * uv[1];
			int b = m_imgOccupancy.data[s];
			//cout << x  << "" << y << " " << b << " " << g << " " << r << " " <<endl;
			if (b > 0)
			{
				//cout << x  << " " << y << ": " << b << " " << g << " " << r << " " <<endl;
				float angle = atan2(y, x);
				while (angle < 0)
				{
					angle += 2 * PI;
				}
				while (angle > 2 * PI)
				{
					angle -= 2 * PI;
				}
				float dist  = sqrt(x*x + y*y);
				int obsID = (int)(angle / (PI/6));
				if (dist < obs[obsID] && dist > D / 2)
				{
					obs[obsID] = dist;
				}
			}
		}
	}
	
	float mindist = 10000;
	VecPosition postemp(0, 0);
	for (int i = 0; i < 12; i++)
	{
	  	//cout << "obsi: " << i << " " << obs[i] << endl;
		if (obs[i] >= D*2)
		{
			VecPosition v = VecPosition::GetVecPositionFromPolar(D, i * PI / 6);
			float dist = (posRobot + v).GetDistanceTo(posTarget);
			//cout << i << " " << dist << endl;
			if (dist < mindist)
			{
				mindist = dist;
				postemp = v;
			}
		}
	}
	
	if (deltaDist > D)
	{
		ToPos(posRobot + postemp);
	}
	else
	{
		ToPos(posTarget);
	}
	
	if (deltaDist < 0.25)
	{
		m_linearSpeed = 0;
		m_angularSpeed = 0;
		res = 1;
	}
	
	return res;
}

int Robot::SetRobotVelocity()
{
	double L = 2;
	double A = 2;
	geometry_msgs::Twist speedMsg;
	speedMsg.linear.x = m_linearSpeed * L;
	speedMsg.angular.z = -m_angularSpeed * A;
	m_speedPub.publish(speedMsg);
	return 0;
}
	
#endif
