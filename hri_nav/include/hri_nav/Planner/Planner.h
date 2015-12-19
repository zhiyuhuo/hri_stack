#ifndef PLANNER_PLANNER_H
#define PLANNER_PLANNER_H

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


#include "../Universal/Header.h"
#include "../Geometry/Header.h"

#define NAVMAPX 400
#define NAVMAPY 400
#define NAVINTERVAL 0.25
#define NAVOFFSETX NAVMAPX/2
#define NAVOFFSETY NAVMAPY/2

using namespace std;

class Planner{
public: 
	Planner();
	~Planner();
	
public:
	cv::Mat m_map;
	
public:
	int PathPlanning();
	//target position sequence {x0,y0,x1,y1...xN,yN}
	VecPosition m_posRobot;
	VecPosition m_posTarget;
	vector<VecPosition>  m_xyPath;
	vector<VecPosition> AstarSearchPath(cv::Mat map, VecPosition posStart, VecPosition posTarget);
	
private:
	double GetPointCost(int u, int v, cv::Mat map, vector<int> uvTarget);
  
	VecPosition PixelToVecPosition(int u, int v);
	vector<int> VecPositionToPixel(VecPosition pos);
};

Planner::Planner()
{
	m_map = cv::Mat::zeros(NAVMAPX, NAVMAPY, CV_8UC1);
}

Planner::~Planner()
{
	m_map.release();
}

int Planner::PathPlanning()
{
	int res = 0;
	VecPosition ps(0, 0);
	VecPosition pt(5, 3);
	vector<VecPosition> steps = AstarSearchPath(m_map, ps, pt);
	res = steps.size();
	return res;
}

vector<VecPosition> Planner::AstarSearchPath(cv::Mat map, VecPosition posStart, VecPosition posTarget)
{
	vector<VecPosition> res;
	
	vector<int> uvS = VecPositionToPixel(posStart);
	vector<int> uvT = VecPositionToPixel(posTarget);
	double IM = sqrt(NAVMAPX*NAVMAPX + NAVMAPY*NAVMAPY);
	
	int u = uvS[0];
	int v = uvS[1];
	double costMin = IM;
	int S = 1;
	
	cout << uvS[0] << " " << uvS[1] << " " << uvT[0] << " " << uvT[1] << endl;
	
	while (u != uvT[0] || v != uvT[1])
	{
		int ii = 0;
		int jj = 0;
		
		for (int i = -S; i <= +S; i++)
		{
			for (int j = -S; j <= +S; j++)
			{
				double cost = GetPointCost(u+i, v+j, map, uvT);
				if (cost < costMin)
				{
					costMin = cost;
					ii = i;
					jj = j;
				}
			}
		}		
		
		u += ii;
		v += jj;
		
		cout << u << " " << v << endl;
	}
	

	return res;
}

double Planner::GetPointCost(int u, int v, cv::Mat map, vector<int> uvTarget)
{
	double res;
	
	double IM = sqrt(NAVMAPX*NAVMAPX + NAVMAPY*NAVMAPY);
	double cost = 0;
	
	cost += sqrt(pow(u-uvTarget[0],2) + pow(v-uvTarget[1],2));
	if (map.data[u + NAVMAPX * v] > 0)
	{
		cost += IM;
	}
	
	res = cost;
	return res;
}

VecPosition Planner::PixelToVecPosition(int u, int v)
{
	double x = ((double)u * NAVINTERVAL) - NAVOFFSETX;
	double y = ((double)v * NAVINTERVAL) - NAVOFFSETX;
	VecPosition res(x, y);
	return res;
}

vector<int> Planner::VecPositionToPixel(VecPosition pos)
{
	vector<int> res(2,0);
	res[0] = (int)((pos.GetX() / NAVINTERVAL) + NAVOFFSETX);
	res[1] = (int)((pos.GetY() / NAVINTERVAL) + NAVOFFSETY);
	return res;
}






#endif


