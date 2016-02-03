#ifndef ACTION_H
#define ACTION_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>

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
	kL = 0.1;
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
	r1 = ToPositionAvoidObstacles(posTarget);
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
	
	for (float x = -1.0; x < 1.0; x += MAP_RESOLUTION)
	{
		for (float y = -1.0; y < 1.0; y += MAP_RESOLUTION)
		{
			vector<int> uv = CoordinateToPixel(x + posRobot.GetX(), y + posRobot.GetY());
			int s = uv[0] + 500 * uv[1];
			int b = m_imgmask.data[3 * s];
			int g = m_imgmask.data[3 * s + 1];
			int r = m_imgmask.data[3 * s + 2];
			//cout << x  << "" << y << " " << b << " " << g << " " << r << " " <<endl;
			if (b > 0 && g > 0 && r > 0 && b != 127 && g != 127 && r != 127 )
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
	
	for (int i = 0; i < m_laserData.size(); i++)
	{
		vector<float> laserUnit = m_laserData[i];
		float angle = laserUnit[0] + m_theta;
		float dist = laserUnit[1];
		while (angle < 0)
		{
			angle += 2 * PI;
		}
		while (angle > 2 * PI)
		{
			angle -= 2 * PI;
		}
		
		int angledir = (int)(angle / (PI/6));
		if (dist < obs[angledir])
		{
			obs[angledir] = dist;
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

int Robot::SearchWide()
{
	int res = 0;
	if (m_action.compare("init") == 0)
	{
		m_turnTarget = m_theta - PI / 3;
		m_action = "search_left";
	}
	
	else if (m_action.compare("search_left") == 0)
	{
		if (ToAngle(m_turnTarget) == 1)
		{
			m_turnTarget = m_theta + PI / 3 * 2;
			m_action = "search_right";
		}
	}
	
	else if (m_action.compare("search_right") == 0)
	{
		if (ToAngle(m_turnTarget) == 1)
		{
			m_turnTarget = m_theta - PI / 3;
			m_action = "move_back";
		}
	}
	
	else if (m_action.compare("move_back") == 0)
	{
		if (ToAngle(m_turnTarget) == 1)
		{
			m_action = "stop";
		}
	}
	
	else if (m_action.compare("stop") == 0)
	{
		res = 1;
	}
	
	return res;
}

int Robot::SearchSpin()
{
	int res = 0;
	if (m_action.compare("init") == 0)
	{
		m_angularSpeed = 0.4;
		m_action = "search_right";
		m_turnTarget = m_theta;
	}
	
	else if (m_action.compare("search_right") == 0)
	{
		float deltaAngle = m_theta - m_turnTarget;
		if (deltaAngle < 0)
		{
			deltaAngle += 2 * PI;
		}
		if (deltaAngle > 2 * PI - PI / 10)
		{
			m_action = "stop";
		}
	}
	
	else if (m_action.compare("stop") == 0)
	{
		m_angularSpeed = 0;
		m_action = "init";
		res = 1;
	}
	
	return res;
}

	
#endif
