#ifndef PERCEPTION_SEENTITY_H_
#define PERCEPTION_SEENTITY_H_

#include "stdio.h"
#include "stdlib.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <math.h>
#include <time.h>
#include <vector>

#include "../Universal/Header.h"


using namespace std;

struct FurnitureDetector{
	string name;
	vector<vector<float> > sv;
	vector<float> a;
	float b;
	vector<float> l;
	vector<float> s;
	vector<float> sh;
};

class SE
{
public:
	SE();
	~SE();	

public: 	
	vector<float> m_pt;
public: 
	float m_highestHeight;
	float m_area;
	float m_dir;
	vector<float> m_centroid;
	vector<float> m_acpt;
	string m_name;
	string m_rawnameStr;
	float m_confidence;

public:
	void ImportPt(vector<int> index, vector<float> pt);
	void ImportPt(vector<float> pt);
	
public:
	void ProceedData(map<string, FurnitureDetector> voxelDetector);
	string Classify(map<string, FurnitureDetector> voxelDetector);

	vector<float> GetPtCentroid(vector<float> pt);
	vector<float> GetAdjustCentroidPt(vector<float> pt, vector<float> centroid);
	
};

SE::SE()
{}
SE::~SE()
{}	

void SE::ImportPt(vector<int> index, vector<float> pt)
{
	for (int i = 0; i < index.size(); i++)
	{
		int id = index[i];
		m_pt.push_back(pt[3*id]);
		m_pt.push_back(pt[3*id+1]);
		m_pt.push_back(pt[3*id+2]);
	}
	//cout << "points number: " << m_pt.size() << endl;
}

void SE::ImportPt(vector<float> pt)
{
	for (int i = 0; i < pt.size(); i++)
	{
		m_pt.push_back(pt[i]);
	}
	//cout << "points number: " << m_pt.size() << endl;
}

void SE::ProceedData(map<string, FurnitureDetector> voxelDetector)
{
  	m_centroid = GetPtCentroid(m_pt);
	m_acpt = GetAdjustCentroidPt(m_pt, m_centroid);  
	
	//cout << "orientation: " << m_dir << endl;
}

string SE::Classify(map<string, FurnitureDetector> voxelDetector)
{
	string name;

	return name;
}

vector<float> SE::GetPtCentroid(vector<float> pt)
{
	float xmin = 100;
	float xmax = -100;
	float ymin = 100;
	float ymax = -100;
	float zmin = 100;
	float zmax = -100;
	
	vector<float> res(3, 0);
	for (int i = 0; i < pt.size()/3; i++)
	{
		float x = pt[3*i];
		float y = pt[3*i+1];
		float z = pt[3*i+2];
		
		if (x > xmax)	xmax = x;
		if (x < xmin)	xmin = x;
		if (y > ymax)	ymax = y;
		if (y < ymin)	ymin = y;
		if (z > zmax)	zmax = z;
		if (z < zmin)	zmin = z;
		
	}
	
// 	cout << xmin << " " << xmax << " " << ymin << " " << ymax << " " << zmin << " " << zmax << endl;
	
	res[0] = (xmin + xmax) / 2;
	res[1] = (ymin + ymax) / 2;
	res[2] = (zmin + zmax) / 2;
	
	//cout << "centroid: " << res[0] << " " << res[1] << " " << res[2] << endl;
	return res;
}

vector<float> SE::GetAdjustCentroidPt(vector<float> pt, vector<float> centroid)
{
	vector<float> res(pt.size(), 0);
	for (int i = 0; i < pt.size()/3; i++)
	{
		res[3*i] = pt[3*i] - centroid[0];
		res[3*i+1] = pt[3*i+1] - centroid[1];
		res[3*i+2] = pt[3*i+2];
	}
	
	return res;
}





#endif








