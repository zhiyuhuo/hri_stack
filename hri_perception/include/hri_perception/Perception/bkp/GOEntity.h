#ifndef PERCEPTION_GOENTITY_H_
#define PERCEPTION_GOENTITY_H_

#include "stdio.h"
#include "stdlib.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <math.h>
#include <time.h>
#include <vector>

#include "../Universal/Header.h"


using namespace std;

class GO
{
public:
	GO();
	~GO();	

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
	void ProceedData(map<string, vector<float> > voxelDetector);
	string Classify(map<string, vector<float> > voxelDetector);

	vector<float> GetPtCentroid(vector<float> pt);
	vector<float> GetAdjustCentroidPt(vector<float> pt, vector<float> centroid);
	void GetVoxel(vector<float> acpt, float voxel[40][40][25]);
	void VectorToVoxel(vector<float> linear, float voxel[40][40][25]);
	vector<float> VoxelToVector(float voxel[40][40][25]);
	string ClassifyVoxel(float voxel[40][40][25], map<string, vector<float> > voxelDetector);
	float MultiplyTwoVoxel(float sample[40][40][25], float detector[40][40][25]);
	float GetDirection();
	float GetHeightFromVoxel(float voxel[40][40][25]);
	float GetAreaFromVoxel(float voxel[40][40][25]);
	float GetRectTableDirection(float voxel[40][40][25]);
	float GetChairFrontDirection(vector<float> centroid, float voxel[40][40][25]);
	float GetChairBackDirection(float voxel[40][40][25]);
	float GetBedDirection(float voxel[40][40][25]);
	
};

GO::GO()
{}
GO::~GO()
{}	

void GO::ImportPt(vector<int> index, vector<float> pt)
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

void GO::ImportPt(vector<float> pt)
{
	for (int i = 0; i < pt.size(); i++)
	{
		m_pt.push_back(pt[i]);
	}
	//cout << "points number: " << m_pt.size() << endl;
}

void GO::ProceedData(map<string, vector<float> > voxelDetector)
{
  	m_centroid = GetPtCentroid(m_pt);
	m_acpt = GetAdjustCentroidPt(m_pt, m_centroid);  
	float voxel[40][40][25] = {};
	GetVoxel(m_acpt, voxel);
	m_highestHeight = GetHeightFromVoxel(voxel);
	m_area = GetAreaFromVoxel(voxel);
	//cout << "centroid:" << m_centroid[0] << " " << m_centroid[1] << "; area: " << m_area << ", height: " << m_highestHeight << endl;; 
	
	string fullname = Classify(voxelDetector);
	//cout << "recog: " << m_name << endl;
	
	if (fullname.find("chair") != string::npos || fullname.find("couch") != string::npos || fullname.find("bed") != string::npos)
	{
	
		if (fullname.find("front") != string::npos)
		{
			m_dir = GetChairFrontDirection(m_centroid, voxel);
		}
		else if (fullname.find("back") != string::npos)
		{
			m_dir = GetChairBackDirection(voxel);
		}
		else
		{
			m_dir = -1;
		}
	
	}
	else
	{
		m_dir = -1;
	}
	//cout << "orientation: " << m_dir << endl;
}

string GO::Classify(map<string, vector<float> > voxelDetector)
{
	float voxel[40][40][25] = {};
	GetVoxel(m_acpt, voxel);
	
	if (m_highestHeight < 0.6)
	{
		voxelDetector.erase("table-high");
		voxelDetector.erase("chair-front");
		voxelDetector.erase("chair-back");
		voxelDetector.erase("couch-front");
	}
	
	if (m_highestHeight >= 0.6)
	{
		voxelDetector.erase("table-low");
	}
	
	if (m_area > 0.6)
	{
		voxelDetector.erase("table-low");
		//voxelDetector.erase("table-high");
		voxelDetector.erase("chair-front");
		voxelDetector.erase("chair-back");
	}
	
	string name = ClassifyVoxel(voxel, voxelDetector);
	string nameStr = "";
	m_rawnameStr = name;
	
	for (int i = 0; i < name.size(); i++)
	{
		if (name[i] != '-')
		{
			nameStr.push_back(name[i]);
		}
		else
		{
			break;
		}
	}

	m_name = nameStr;
	//cout << m_name << endl;
	return name;
}

vector<float> GO::GetPtCentroid(vector<float> pt)
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

vector<float> GO::GetAdjustCentroidPt(vector<float> pt, vector<float> centroid)
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

void GO::GetVoxel(vector<float> acpt, float voxel[40][40][25])
{
	for (int i = 0; i < acpt.size() / 3; i++)
	{
		int XI = (int)(acpt[3*i] / 0.05) + 20;
		int YI = (int)(acpt[3*i+1] / 0.05) + 20;
		int ZI = (int)(acpt[3*i+2] / 0.05);
		
		if (XI > 0 && YI > 0 && ZI > 0 && XI < 40 && YI < 40 && ZI < 25)
		{
			voxel[XI][YI][ZI] = 1;
		}		
	}

}

void GO::VectorToVoxel(vector<float> linear, float voxel[40][40][25])
{
	for (int i = 0; i < 40; i++)
	{
		for (int j = 0; j < 40; j++)
		{
			for (int k = 0; k < 25; k++)
			{
				voxel[i][j][k] = linear[i*40*25+j*25+k];
			}
		}
	}
}

vector<float> GO::VoxelToVector(float voxel[40][40][25])
{
	vector<float> f(40*40*25, 0);
	for (int i = 0; i < 40; i++)
	{
		for (int j = 0; j < 40; j++)
		{
			for (int k = 0; k < 25; k++)
			{
				f[i*40*25+j*25+k] = voxel[i][j][k];
			}
		}
	}
	return f;
}

string GO::ClassifyVoxel(float voxel[40][40][25], map<string, vector<float> > voxelDetector)
{
	map<string, vector<float> >::iterator iter;
	float mMax = -10000;
	string classMax = "";
	for (iter = voxelDetector.begin(); iter != voxelDetector.end(); ++iter)
	{
		string keystr = iter->first;
		vector<float> detector = iter->second;
		float dct[40][40][25] = {};
		VectorToVoxel(detector, dct);
		float m = MultiplyTwoVoxel(voxel, dct);
		//cout << "	 = " << keystr << " " << m << endl;
		if (m > mMax)
		{
			mMax = m;
			classMax = keystr;
		}
	}
	
	m_confidence = mMax;
	if (mMax <= 0)
	{
		classMax = "unknown";
	}
	//cout << "-class: " << classMax << " " << mMax << endl;
	return classMax;
}

float GO::MultiplyTwoVoxel(float sample[40][40][25], float detector[40][40][25])
{
	float res = 0;

	float sumresp = 0;
	float sumdetector = 0;
	for (int i = 0; i < 40; i++)
	{
		for (int j = 0; j < 40; j++)
		{
			for (int k = 0; k < 25; k++)
			{
				sumresp += sample[i][j][k] * detector[i][j][k];
				sumdetector += detector[i][j][k];
			}
		}
	}
	
	res = sumresp / sumdetector;
	return res;
}

float GO::GetDirection()
{
	float res = 0;
	return res;
}

float GO::GetHeightFromVoxel(float voxel[40][40][25])
{
	float res = 0;
	
	float highest = 0;	
	for (int k = 5; k < 25; k++)
	{
		float areak = 0;
		for (int i = 0; i < 40; i++)
		{
			for (int j = 0; j < 40; j++)
			{
				if (voxel[i][j][k] > 0)
				{
					areak++;
				}
			}
		}
		
		if (areak > 10)
		{
			highest = k;
		}
	}
	
	res = highest * 0.05;
	return res;
}

float GO::GetAreaFromVoxel(float voxel[40][40][25])
{
	float res = 0;
	
	float occupy = 0;	
	float map[40][40] = {};
	for (int k = 0; k < 25; k++)
	{
		for (int i = 0; i < 40; i++)
		{
			for (int j = 0; j < 40; j++)
			{
				if (voxel[i][j][k] > 0 && map[j][i] == 0)
				{
					occupy++;
					map[j][i]++;
				}
			}
		}
	}
	
	res = occupy * (0.05*0.05);
	return res;	
}

float GO::GetRectTableDirection(float voxel[40][40][25])
{
	float res = 0;
	return res;
}

float GO::GetChairFrontDirection(vector<float> centroid, float voxel[40][40][25])
{
  	float res = 0;
	vector<float> vecPlane;
	vector<float> vecTop;
	float maxArea = 0;
	float maxHeight = 0;
	float highest = 0;
	
	for (int k = 5; k < 25 - 1; k++)
	{
		float areakk[2] = {0, 0};
		for (int kk = k; kk < k+2; kk++)
		{
			for (int i = 0; i < 40; i++)
			{
				for (int j = 0; j < 40; j++)
				{
					if (voxel[i][j][kk] > 0)
					{
						areakk[kk-k]++;
					}
				}
			}
		}

		if (areakk[0] + areakk[1] > maxArea)
		{
			maxArea = areakk[0] + areakk[1];
			maxHeight = (k * areakk[0] + (k+1) * areakk[1]) / (areakk[0] + areakk[1]);
		}
		
		if (areakk[1] > 10)
		{
			highest = k + 1;
		}
	}
	
	float map[40][40] = {0};
	
	for (int i = 20-1; i <= 20+1; i++)
	{
		for (int j = 20-1; j <= 20+1; j++)
		{
			map[j][i] = 2;
			vecPlane.push_back(j);
			vecPlane.push_back(i);
		}
	}
	
	for (int k = (int)highest; k < 25; k++)
	{
		for (int i = 0; i < 40; i++)
		{
			for (int j = 0; j < 40; j++)
			{
				if (voxel[j][i][k] > 0 && map[j][i] == 0)
				{
					map[j][i] = 1;
					vecTop.push_back(j);
					vecTop.push_back(i);
				}
			}
		}
	}
	
	float mindist = 100;
	vector<float> mindistangle;
	for (int i = 0; i < vecTop.size()/2; i++)
	{
		for (int j = 0; j < vecPlane.size()/2; j++)
		{
			float dist = sqrt(pow(vecTop[2*i]-vecPlane[2*j], 2) + pow(vecTop[2*i+1]-vecPlane[2*j+1], 2));
			if (dist <= mindist)
			{
				if (dist < mindist)
				{
					mindistangle.clear();
				}
				mindist = dist;
				float angle = atan2(vecPlane[2*j+1]-vecTop[2*i+1], vecPlane[2*j]-vecTop[2*i]);
				angle = NormalizeDirection(angle);
				mindistangle.push_back(angle); 
			}
		}
	}
	
	float a = 0;
	for (int i = 0; i < mindistangle.size(); i++)
	{
		a += mindistangle[i];
	}
	a /= mindistangle.size();
	//cout << a * 180 / PI << endl;
	res = a;
	
	return res;
}

float GO::GetChairBackDirection(float voxel[40][40][25])
{
  	float res = 0;
	
	vector<float> vecTop;
	float maxArea = 0;
	float maxHeight = 0;
	float highest = 0;
	
	for (int k = 5; k < 25 - 1; k++)
	{
		float areak = 0;
		for (int i = 0; i < 40; i++)
		{
			for (int j = 0; j < 40; j++)
			{
				if (voxel[i][j][k] > 0)
				{
					areak++;
				}
			}
		}
		
		if (areak > 10)
		{
			highest = k;
		}
	}
	
	float map[40][40] = {0};
	for (int k = (int)highest; k < 25; k++)
	{
		for (int i = 0; i < 40; i++)
		{
			for (int j = 0; j < 40; j++)
			{
				if (voxel[j][i][k] > 0 && map[j][i] == 0)
				{
					map[j][i] = 1;
					vecTop.push_back(j);
					vecTop.push_back(i);
				}
			}
		}
	}

	float maxdist = 0;
	float maxdistangle = 0;
	for (int i = 0; i < vecTop.size()/2; i++)
	{
		for (int j = 0; j < vecTop.size()/2; j++)
		{
			float dist = sqrt(pow(vecTop[2*i]-vecTop[2*j], 2) + pow(vecTop[2*i+1]-vecTop[2*j+1], 2));
			if (dist > maxdist)
			{
				maxdist = dist;
				float angle = atan2(vecTop[2*j+1]-vecTop[2*i+1], vecTop[2*j]-vecTop[2*i]);
				maxdistangle = NormalizeDirection(angle);
			}
		}
	}

	float a = maxdistangle - PI/2;
	a = NormalizeDirection(a);
	if (a > PI)
	{
		a -= PI;
	}
	//cout << "a: " << a * 180 / PI << endl;
	res = a;
	
	return res;
}

float GO::GetBedDirection(float voxel[40][40][25])
{
  	float res = 0;
	return res;
}



#endif








