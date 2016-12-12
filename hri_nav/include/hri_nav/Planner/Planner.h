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
#include "dijkstra.h"

using namespace std;

class Planner{
public: 
	Planner();
	~Planner();
	
public:
	int m_width;
	int m_height;
	double m_resolution;
	VecPosition m_origin;
	
	cv::Mat m_map;
	
public:
	int PathPlanningTest();
	//target position sequence {x0,y0,x1,y1...xN,yN}
	VecPosition m_posRobot;
	VecPosition m_posTarget;
	vector<VecPosition>  m_xyPath;
	int SetMap(int width, int height, double resolution, VecPosition origin, vector<uint8_t> data);
	vector<VecPosition> GetPlan(VecPosition posStart, VecPosition posTarget);
	vector<VecPosition> AstarSearchPath(cv::Mat map, VecPosition posStart, VecPosition posTarget);
	vector<VecPosition> DijkstraSearchPath(cv::Mat map, VecPosition posStart, VecPosition posTarget);
	
private:
	double GetPointCost(int u, int v, cv::Mat map, vector<int> uvTarget);
  
	VecPosition PixelToVecPosition(int u, int v);
	vector<int> VecPositionToPixel(VecPosition pos);
};

Planner::Planner()
{	
	m_map = cv::Mat::zeros(100, 100, CV_8UC1);
}

Planner::~Planner()
{
	m_map.release();
}

int Planner::PathPlanningTest()
{
	int res = 0;
	VecPosition ps(0, 0);
	VecPosition pt(5, 3);
	vector<VecPosition> steps = AstarSearchPath(m_map, ps, pt);
	res = steps.size();
	return res;
}

int Planner::SetMap(int width, int height, double resolution, VecPosition origin, vector<uint8_t> data)
{
	m_map.release();
	m_width = width;
	m_height = height;
	m_resolution = resolution;
	m_origin = origin;
	m_map = cv::Mat::zeros(m_width, m_height, CV_8UC1);
	for (int i = 0; i < m_width; i++)
	{
		for (int j = 0; j < m_height; j++)
		{
			m_map.data[i+j*m_width] = (unsigned char)(data[i+j*m_width]);
		}
	}
	cv::imshow("m_map", m_map);
	cv::waitKey(1000);
	return 0;
}

vector<VecPosition> Planner::GetPlan(VecPosition posStart, VecPosition posTarget)
{	
// 	vector<VecPosition> path1 = AstarSearchPath(m_map, posStart, posTarget);
// 	cout << "path 1_: " << path1.size() << endl;
// 	for (int i = 0; i < path1.size(); i++)
// 	{
// 		cout << path1[i].GetX() << "  " << path1[i].GetY() << endl;
// 	}
	
	
	vector<VecPosition> path2 = DijkstraSearchPath(m_map, posStart, posTarget);
	cout << "path 2 : " << path2.size() << endl;
	for (int i = 0; i < path2.size(); i++)
	{
		cout << path2[i].GetX() << "  " << path2[i].GetY() << endl;
	}
	
	return path2;
}

vector<VecPosition> Planner::AstarSearchPath(cv::Mat map, VecPosition posStart, VecPosition posTarget)
{
	vector<VecPosition> res;
	
	vector<int> uvS = VecPositionToPixel(posStart);
	vector<int> uvT = VecPositionToPixel(posTarget);
	double distMax = sqrt(map.cols*map.cols + map.rows*map.rows);
	
	int u = uvS[0];
	int v = uvS[1];
	double costMin = distMax;
	int S = 1;
	
	//cout << uvS[0] << " " << uvS[1] << " " << uvT[0] << " " << uvT[1] << endl;
	
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
		
		//cout << u << " " << v << endl;
		VecPosition pm = PixelToVecPosition(u, v);
		if (pm.GetDistanceTo(posTarget) > m_resolution * sqrt(2))
		{
			res.push_back(pm);
		}
		else
		{
			res.push_back(posTarget);
			break;
		}
	}
	

	return res;
}

vector<VecPosition> Planner::DijkstraSearchPath(cv::Mat map, VecPosition posStart, VecPosition posTarget)
{
	cv::imshow("m_map", map);
	cv::waitKey(30);
	vector<VecPosition> res;
	vector<int> uvS = VecPositionToPixel(posStart);
	vector<int> uvT = VecPositionToPixel(posTarget);
	vector<cv::Point2i> path;
	
	// Start top right
	cv::Point2i start(uvS[0], uvS[1]);
	// Goal bottom left
	cv::Point2i goal(uvT[0], uvT[1]);
	cout << uvS[0] << "  " << uvS[1] << "  " << map.at<uint8_t>(uvS[1], uvS[0]) << endl;
	cout << uvT[0] << "  " << uvT[1] << "  " << map.at<uint8_t>(uvT[1], uvT[0]) << endl;

	findPathViaDijkstra(map, start, goal, path);
	cout << "here path has " << path.size() << " nodes" << endl;
	for (int i = 0; i < path.size(); i++)
	{
		VecPosition p = PixelToVecPosition(path[i].x, path[i].y);
		res.push_back(p);
	}
	
	return res;
}

double Planner::GetPointCost(int u, int v, cv::Mat map, vector<int> uvTarget)
{
	double res;
	
	double distMax = sqrt(map.cols*map.cols + map.rows*map.rows);
	double cost = 0;
	
	cost += sqrt(pow(u-uvTarget[0],2) + pow(v-uvTarget[1],2));
	if (map.data[u + map.cols * v] > 0)
	{
		cost += distMax;
	}
	
	res = cost;
	return res;
}

VecPosition Planner::PixelToVecPosition(int u, int v)
{
	double x = ((double)u * m_resolution) + m_origin.GetX();
	double y = ((double)v * m_resolution) + m_origin.GetY();
	VecPosition res(x, y);
	return res;
}

vector<int> Planner::VecPositionToPixel(VecPosition pos)
{
	vector<int> res(2,0);
	res[0] = (int)((pos.GetX() - m_origin.GetX()) / m_resolution);
	res[1] = (int)((pos.GetY() - m_origin.GetY()) / m_resolution);
	return res;
}

#endif


