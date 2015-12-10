#ifndef PERCEPTION_PERCEPTION_H
#define PERCEPTION_PERCEPTION_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../Universal/Header.h"

#include "Header.h"


using namespace std;

struct Ent{
	vector<double> vec;
	float dir;
	string name;
	float x;
	float y;
};

class Perception
{
public:
	Perception();
	~Perception();
  
public:
	Mat m_imgScene;
	vector<float> m_pt;
	map<string, vector<float> > m_voxelDetector;
	vector<Ent> m_GOList;
	
public: 
  	int ReadVoxelDetector(string folderName);
	int ImportKinectData(Mat imgRGB, vector<float> pts);
	int Process();
	Ent GenerateEnt(GO g);
};

Perception::Perception()
{
	m_imgScene = Mat::zeros( IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
}

Perception::~Perception()
{
  
}

int Perception::ReadVoxelDetector(string folderName)
{	
	int res = 0;
	vector<float> f;
	f = ReadFloatVectorFile(folderName + "/V/table-low.v");
	m_voxelDetector["table-low"] = f;
	f = ReadFloatVectorFile(folderName + "/V/table-high.v");
	m_voxelDetector["table-high"] = f;
	f = ReadFloatVectorFile(folderName + "/V/chair-front.v");
	m_voxelDetector["chair-front"] = f;
	f = ReadFloatVectorFile(folderName + "/V/chair-back.v");
	m_voxelDetector["chair-back"] = f;
	f = ReadFloatVectorFile(folderName + "/V/bed.v");
	m_voxelDetector["bed"] = f;
	f = ReadFloatVectorFile(folderName + "/V/couch-front.v");
	m_voxelDetector["couch-front"] = f;
	
	return res;
}

int Perception::ImportKinectData(Mat imgRGB, vector<float> pts)
{
	int res = 0;
	memcpy(m_imgScene.data, imgRGB.data, IMG_SIZE3);
	m_pt = pts;
	return res;
}

int Perception::Process()
{
	m_GOList.clear();
  
	int res = 0;
	Mat localMap = Mat::zeros(LOCALMAP_HEIGHT, LOCALMAP_WIDTH, CV_8UC1);
	Mat labelLocalMap = Mat::zeros(LOCALMAP_HEIGHT, LOCALMAP_WIDTH, CV_8UC1);
	
	
	BuildLocalMap(m_pt, localMap);
	BwLabel(localMap, labelLocalMap);
	SetBwMapBackgroundZero(labelLocalMap);
	
	imshow("LocalMap", localMap);
	waitKey(1);

	//Find the number of blobs
	int Num = 0;
	for (int i = 0; i < LOCALMAP_SIZE; i++)
	{
		int blobID = labelLocalMap.data[i];
		if (blobID > Num)
		{
			Num = blobID;
		}
	}
	vector<int> blobToProcess(Num+1, 0);
	for (int i = 0; i < LOCALMAP_SIZE; i++)
	{
		int blobID = labelLocalMap.data[i];
		if (blobID > 0)
		{
			blobToProcess[blobID]++;
		}
	}
	
	//Generate Furniture Samples and Add them to Robot Vars.
	int ct = 0;
	for (int n = 1; n <= Num; n++)
	{	
		float area = blobToProcess[n] * (LOCALMAP_RESOLUTION * LOCALMAP_RESOLUTION);
		if (area > 0.1)
		{
			vector<int> targetIndex;
			for (int i = 0; i < IMG_SIZE; i++)
			{
			  
				float x, y, z;
				x = m_pt[3*i];
				y = m_pt[3*i+1];
				z = m_pt[3*i+2];
				int xx, yy;
				xx = (int)((x + LOCALMAP_X / 2) / LOCALMAP_RESOLUTION);
				yy = (int)(y / LOCALMAP_RESOLUTION);
				if (xx<LOCALMAP_WIDTH && xx>=0 && yy<LOCALMAP_HEIGHT && yy>=0 && z > 0.2)
				{
					int gi = (int)(LOCALMAP_WIDTH * yy + xx);
					if (labelLocalMap.data[gi] == n)
					{
						targetIndex.push_back(i);
					}
				}
			}

			///Furniture recognition start
			GO g;
			g.ImportPt(targetIndex, m_pt);
			g.ProceedData(m_voxelDetector);
			///Furniture recognition end
			
			//cout << g.m_centroid[0] << ", " << g.m_centroid[1] << endl;
			if (g.m_centroid[0] < LOCALMAP_X/2-1 && g.m_centroid[0] > -LOCALMAP_X/2+1 && g.m_centroid[1] > 0.5 && g.m_centroid[1] < LOCALMAP_Y-1)
			{
			
				Ent en = GenerateEnt(g);
				cout << en.name << "	" << en.x << ", " << en.y << endl;
				m_GOList.push_back(en);	
				res++;
			}
		}
	}

	return res;
}

Ent Perception::GenerateEnt(GO g)
{
	Ent en;
	
	en.name = g.m_name;
	en.dir = g.m_dir;
	en.x = g.m_centroid[0];
	en.y = g.m_centroid[1];
	
	vector<double> vec;
	float map[(int)(LOCALMAP_HEIGHT)][(int)(LOCALMAP_WIDTH)] = {};
	for (int i = 0; i < g.m_pt.size()/3; i++)
	{
		float lx = g.m_pt[3*i];
		float ly = g.m_pt[3*i+1];

		int x = (int)(lx / LOCALMAP_RESOLUTION + LOCALMAP_WIDTH/2);
		int y = (int)(ly / LOCALMAP_RESOLUTION);
		if (map[y][x] == 0)
		{
			vec.push_back((double)((x - LOCALMAP_WIDTH/2)*LOCALMAP_RESOLUTION));
			vec.push_back((double)(y*LOCALMAP_RESOLUTION));
			map[y][x] = 1;
		}
	}

	//delete map;
	en.vec = vec;
	return en;
}

#endif