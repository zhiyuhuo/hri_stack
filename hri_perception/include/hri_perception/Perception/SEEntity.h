#ifndef PERCEPTION_SEENTITY_H_
#define PERCEPTION_SEENTITY_H_

#include "stdio.h"
#include "stdlib.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <math.h>
#include <time.h>
#include <vector>

#include <boost/thread/thread.hpp>
// #include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>

#include "../Universal/Header.h"


using namespace std;

struct FurnitureDetector{
  
	string name;
	vector<vector<float> > sv;
	vector<float> a;
	float b;
	vector<float> l;
	vector<float> sf;
	vector<float> sh;
	vector<float> anglew;
	
	float sigma;
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
	void ProceedData(map<string, FurnitureDetector> detectors);
	
	vector<float> GetPtCentroid(vector<float> pt);
	vector<float> GetAdjustCentroidPt(vector<float> pt, vector<float> centroid);
	
	//classification
	vector<float> BuildDHTiltFeature(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> normals);
	float GetTilt(float nx, float ny, float nz);	
	string Classify(vector<float> feature, map<string, FurnitureDetector> detectors);
        float LogisticClassifyOneSample(vector<float> feature, FurnitureDetector model);
	float KernelRBF(vector<float> u, vector<float> v, float sigma);
	
	//orientation 
	float GetDir(float nx, float ny, float nz);
	vector<float> BuildDHAngleFeature(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> normals);
	float GetOrientation(vector<float> feature, FurnitureDetector detector);
};

SE::SE()
{
	m_name = "";
	m_dir = -1;
}
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

void SE::ProceedData(map<string, FurnitureDetector> detectors)
{
	//load sample
  	m_centroid = GetPtCentroid(m_pt);
	m_acpt = GetAdjustCentroidPt(m_pt, m_centroid);  
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZ> ());
	cloudin->width    = m_acpt.size()/3;
	cloudin->height   = 1;
	cloudin->is_dense = false;
	cloudin->points.resize (cloudin->width * cloudin->height);
	
	for (int i = 0; i < m_acpt.size()/3; i++)
	{
		cloudin->points[i].x = m_acpt[3*i];
		cloudin->points[i].y = m_acpt[3*i+1];
		cloudin->points[i].z = m_acpt[3*i+2];
	}

	
	//voxelize pc
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloudin);
	sor.setLeafSize (0.02f, 0.02f, 0.02f);
	sor.filter (*cloudin);
	
	pcl::PointCloud<pcl::PointXYZ> cloud = *cloudin;
	
	//get normal
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloudin);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr normalsin (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (0.03);
	ne.compute (*normalsin);	
	pcl::PointCloud<pcl::Normal> normals = *normalsin; 
	
	//get feature
	vector<float> featureTilt = BuildDHTiltFeature(cloud, normals);
	
	//////////////classify get the name of the sample;
	m_rawnameStr = Classify(featureTilt, detectors);
	m_name = "";
	int i = 0;
	char c = m_rawnameStr[i];
	while (i < m_rawnameStr.size() && c != '_')
	{
		m_name.push_back(c);
		c = m_rawnameStr[++i];
	}
	
	//////////////get the direction of the sample;
	vector<float> featureAngle = BuildDHAngleFeature(cloud, normals);
	//m_dir = GetOrientation(featureAngle, detectors[m_rawnameStr]);
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

vector<float> SE::BuildDHTiltFeature(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> normals)
{
	vector<float> res(100, -1);
	
	vector<float> M(100, 0);
	vector<float> T(100, 0);
	
	float F = 0.1;
	int ID, D, H;
	float tilt;
	for (int i = 0; i < normals.width; i++)
	{
		if (!isnan(normals.points[i].normal_x) && !isnan(normals.points[i].normal_y) && !isnan(normals.points[i].normal_z))
		{
			tilt = GetTilt(normals.points[i].normal_x, normals.points[i].normal_y, normals.points[i].normal_z);
			D = (int)(sqrt(cloud.points[i].x * cloud.points[i].x + cloud.points[i].y * cloud.points[i].y) / F);
			H = (int)(cloud.points[i].z / F);
			if (D >= 0 && D < 10 && H > 0 && H < 10)
			{
				ID = D + H * 10;
				M[ID] += tilt;
				T[ID] ++;
			}
		}
	}
	
	for (int i = 0; i < res.size(); i++)
	{
		if (T[i] > 0)
		{
			res[i] = M[i] / T[i];
		}
	}
	
	return res;
}

float SE::GetTilt(float nx, float ny, float nz)
{
	return fabs(nz) / sqrt(nx*nx + ny*ny + nz*nz);
}

string SE::Classify(vector<float> feature, map<string, FurnitureDetector> detectors)
{
	string name = "unknown";
	map<string, FurnitureDetector>::iterator iter;
	for (iter = detectors.begin(); iter != detectors.end(); ++iter)
	{
		string keystr = iter->first;
		FurnitureDetector dct = iter->second;
		
		float r = LogisticClassifyOneSample(feature, dct);
// 		cout << keystr << ": " << r << endl;
		
		if (r > 0)
		{
			name = keystr;
			break;
		}
	}
	
	return name;
}

float SE::LogisticClassifyOneSample(vector<float> feature, FurnitureDetector model)
{
	float res;
	//cout << model.name << endl;
	vector<float> sample(feature.size(), 0);
	for (int i = 0; i < sample.size(); i++)
	{
		sample[i] = (feature[i] + model.sh[i]) * model.sf[i];
	}  
	  
	float r = 0;
	for (int i = 0; i < model.sv.size(); i++)
	{
		if (i == 0)
		{
			vector<float> sv = model.sv[i];
// 			for (int k = 0; k < sv.size(); k++)
// 			{
// 				cout << "_" << k << " " << feature[k] << "   " << sample[k] << "   " << sv[k] << endl;
// 			}
		}
		float kval = KernelRBF(sample, model.sv[i], model.sigma);
		//cout << i << " " << kval << endl;
		r += (-1) * model.a[i] * kval;
	}
	
	r -= model.b;
	res = (r>0)?1:-1;
	
	return res;
}


float SE::KernelRBF(vector<float> u, vector<float> v, float sigma)
{	
	float res;
    
        vector<float> d2vec(u.size(), 0);
	float d2sum = 0;
	for (int i = 0; i < u.size(); i++)
	{
		d2vec[i] = (u[i] - v[i]) * (u[i] - v[i]);
	}
	for (int i = 0; i < d2vec.size(); i++)
	{
		d2sum += d2vec[i];
	}
	
	d2sum = pow(sqrt(d2sum), 2);
	
	res = exp( -1/(2*sigma*sigma) * d2sum);
	
	return res;
}

float SE::GetDir(float nx, float ny, float nz)
{
	float th = atan2(ny, nx);
	if (th < 0)
	{
		th += 2 * PI;
	}
	
	return th;
}

vector<float> SE::BuildDHAngleFeature(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> normals)
{
	vector<float> res(100, -1);
	
	vector<float> X(100, 0);
	vector<float> Y(100, 0);
	vector<float> M(100, 0);
	vector<float> T(100, 0);
	
	float F = 0.1;
	int ID, D, H;
	float th;
	for (int i = 0; i < normals.width; i++)
	{
		if (!isnan(normals.points[i].normal_x) && !isnan(normals.points[i].normal_y) && !isnan(normals.points[i].normal_z))
		{
			if (GetTilt(normals.points[i].normal_x, normals.points[i].normal_y, normals.points[i].normal_z) < 0.5)
			{
				th = GetDir(normals.points[i].normal_x, normals.points[i].normal_y, normals.points[i].normal_z);
				float signth = normals.points[i].normal_x * ( -cloud.points[i].x) + normals.points[i].normal_y * (-cloud.points[i].y); 
				if (signth < 0)
				{
					th = th - PI;
				}
				if (th < 0)
				{
					th += 2 * PI;
				}
				
				D = (int)(sqrt(cloud.points[i].x * cloud.points[i].x + cloud.points[i].y * cloud.points[i].y) / F);
				H = (int)(cloud.points[i].z / F);
				if (D >= 0 && D < 10 && H > 0 && H < 10)
				{
					ID = D + H * 10;
					X[ID] += cos(th);
					Y[ID] += sin(th);
					T[ID] ++;
				}
			}
		}
	}
	
	for (int i = 0; i < res.size(); i++)
	{
		if (T[i] > 0)
		{
			M[i] = atan2(Y[i] / T[i], X[i] / T[i]);
			if (M[i] < 0)
			{
				M[i] += 2 * PI;
			}
			res[i] = M[i];
		}
	}
	
	return res;
}

float SE::GetOrientation(vector<float> feature, FurnitureDetector detector)
{
	float res = -1;
	
	cout << detector.anglew.size() << endl;
	if (detector.anglew.size() > 0)
	{
	  	float angle = 0;
		float w = 0;
	  
		int i;
		for (i = 0; i < detector.anglew.size(); i++)
		{
			cout << i << ":    " << feature[i] << ",   " << detector.anglew[i] << endl;
			if (detector.anglew[i] > 0.5 && feature[i] > 0)
			{
				angle += feature[i] * detector.anglew[i];
				w += detector.anglew[i];
			}
		}
		
		angle /= w;
		res = angle;
	}
	
	
	return res;
}

#endif








