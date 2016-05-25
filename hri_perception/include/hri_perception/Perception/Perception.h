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

#define DRAW_TAG	

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
	cv::Mat m_imgTag;
	cv::Mat m_imgScene;
	cv::Mat m_imgLocalMap;
	vector<float> m_pt;
	map<string, FurnitureDetector> m_detectorSet;
	vector<Ent> m_SEList;
	
public: 
	FurnitureDetector ReadOneFurnitureDetector(string rootDir, string name);
  	int ReadDetectors(string rootDir);
	int ImportKinectData(cv::Mat imgRGB, vector<float> pts);
	int Process();
	Ent GenerateEnt(SE g);
	int DrawImageTag(vector<int> index, string name);
	
private:
	vector<vector<float> > LoadVectorVectorFloat(string fileName);
	vector<float> LoadVectorFloat(string fileName);
	float LoadFloat(string fileName);
};

Perception::Perception()
{
	m_imgScene = cv::Mat::zeros( IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
	m_imgTag = cv::Mat::zeros( IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
	m_imgLocalMap = cv::Mat::zeros(LOCALMAP_HEIGHT, LOCALMAP_WIDTH, CV_8UC1);
}

Perception::~Perception()
{
  
}

FurnitureDetector Perception::ReadOneFurnitureDetector(string rootDir, string name)
{
	FurnitureDetector res;
	res.name = name;
	res.sv = LoadVectorVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/supportvectors.txt");
	res.a = LoadVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/alpha.txt");
	res.b = LoadFloat(rootDir + "/FurnitureModelsForRos/" + name + "/bias.txt");
	res.l = LoadVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/group.txt");
	res.sf = LoadVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/scalefactor.txt");
	res.sh = LoadVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/shift.txt");
	res.sigma = LoadFloat(rootDir + "/FurnitureModelsForRos/" + name + "/sigma.txt");
	res.anglew = LoadVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/anglew.txt");
	return res;
}

int Perception::ReadDetectors(string rootDir)
{	
	int res = 0;
	
	FurnitureDetector fd;
	fd = ReadOneFurnitureDetector(rootDir, "table_low");
	m_detectorSet["table_low"] = fd;
	fd = ReadOneFurnitureDetector(rootDir, "table_high");
	m_detectorSet["table_high"] = fd;
	fd = ReadOneFurnitureDetector(rootDir, "chair");
	m_detectorSet["chair"] = fd;
	fd = ReadOneFurnitureDetector(rootDir, "couch");
	m_detectorSet["couch"] = fd;
	fd = ReadOneFurnitureDetector(rootDir, "bed");
	m_detectorSet["bed"] = fd;
	
	
	return res;
}

int Perception::ImportKinectData(cv::Mat imgRGB, vector<float> pts)
{
	int res = 0;
	memcpy(m_imgScene.data, imgRGB.data, IMG_SIZE3);
	m_pt = pts;
	return res;
}

int Perception::Process()
{
	m_SEList.clear();
  
	int res = 0;
	cv::Mat labelLocalMap = cv::Mat::zeros(LOCALMAP_HEIGHT, LOCALMAP_WIDTH, CV_8UC1);
	
	
	BuildLocalMap(m_pt, m_imgLocalMap);
	BwLabel(m_imgLocalMap, labelLocalMap);
	SetBwMapBackgroundZero(labelLocalMap);
	
	//cv::imshow("LocalMap", localMap);
	//cv::waitKey(1);

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
	cout << "Processing Data...: " << endl;
        #ifdef DRAW_TAG
	m_imgTag.setTo(cv::Scalar(0, 0, 0));
	#endif
	for (int n = 1; n <= Num; n++)
	{	
		float area = blobToProcess[n] * (LOCALMAP_RESOLUTION * LOCALMAP_RESOLUTION);
		if (area > 0.1)
		{
// 			cout << " -sample: " << n << "     ";
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
			
			SE g;
			g.ImportPt(targetIndex, m_pt);
			g.ProceedData(m_detectorSet);
			
			///Furniture recognition end
			
			//cout << "  " << g.m_name << ":   " << g.m_centroid[0] << ", " << g.m_centroid[1] << endl;
			if (g.m_centroid[0] < LOCALMAP_X/2-1.5 && g.m_centroid[0] > -LOCALMAP_X/2+1.5 && g.m_centroid[1] > 0.5 && g.m_centroid[1] < LOCALMAP_Y-0.5)
			{
			  
			        #ifdef DRAW_TAG
				DrawImageTag(targetIndex, g.m_name);
				#endif
			
				Ent en = GenerateEnt(g);
				cout << "   " << en.name << "	" << en.x << ", " << en.y << ", " << en.dir << " - " << en.vec.size()/2 << endl;
				m_SEList.push_back(en);	
				res++;
			}
			else
			{
				cout << endl;
			}
		}
	}

	return res;
}

Ent Perception::GenerateEnt(SE g)
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

int Perception::DrawImageTag(vector<int> index, string name)
{
	unsigned char r,g,b;
	vector<string> keys;
	for (map<string, FurnitureDetector>::iterator i = m_detectorSet.begin(); i != m_detectorSet.end(); ++i)
	{
		keys.push_back(i->first);
	}
	
	map<string, cv::Scalar> colors; 
	colors["table"] = cv::Scalar(255, 0, 0);
	colors["chair"] = cv::Scalar(0, 255, 0);
	colors["couch"] = cv::Scalar(0, 0, 255);
	colors["bed"] = cv::Scalar(255, 255, 0);
	colors["unknown"] = cv::Scalar(150, 150, 150);

	if (colors.count(name) > 0)
	{
		cv::Scalar rgb = colors[name];
		r = rgb.val[0];
		g = rgb.val[1];
		b = rgb.val[2];
		
		int L = index.size();
		for (int i = 0; i < L; i++) 
		{
			m_imgTag.data[3*index[i]] = r; 
			m_imgTag.data[3*index[i] + 1] = g; 
			m_imgTag.data[3*index[i] + 2] = b; 
		}
	}
	
	//cv::waitKey(1);
	//imshow("image tag", m_imgTag);
	return 0;
}

vector<vector<float> > Perception::LoadVectorVectorFloat(string fileName)
{
	vector<vector<float> > res;

	string line;
	ifstream fs (fileName.c_str());
	cout << fileName << endl;
	if (fs.is_open())
	{
		int k = 0;
		while (fs.good())
		{
			vector<float> vf;
			getline(fs,line);
			//cout << "line: " << k << ": " << line << endl;
			string s = "";
			int t = 0;
			char c;
			float f;
			while (t < line.size())
			{
				c = line[t];
				if (c == ',')
				{
					f = atof(s.c_str());
					vf.push_back(f);
					s = "";
				}
				else
				{
					s.push_back(c);
				}
				t++;
			}
			f = atof(s.c_str());
			vf.push_back(f);
			
			if (line.size() > 0)
			{
				res.push_back(vf);
			}
			k++;
		}
		
		cout << res.size() << " " << res[0].size() << " " << res[res.size()-1].size() << endl;
		fs.close();
	}
	else
	{
		cout << fileName << " - cannot open" << endl;
		return res;
	}
	
	return res;
}

vector<float> Perception::LoadVectorFloat(string fileName)
{
	cout << fileName << endl;
  
	vector<float> res;
	
	char c;
	float f;
	string s = "";
	FILE *file;
	file = fopen(fileName.c_str(), "r");
	if (file) 
	{
		while ((c = getc(file)) != EOF)
		{
			if (c == 45 || c == 46 || (c >= 48 && c < 58))
			{
				s.push_back(c);
			}
			else
			{
				f = atof(s.c_str());
				//cout << f << endl;
				res.push_back(f);
				s = "";
			}
		}

		fclose(file);
	}
	else
	{
		cout << fileName << " - cannot open" << endl;
		return res;
	}

	cout << res.size() << endl;
	return res;
}

float Perception::LoadFloat(string fileName)
{
	cout << fileName << endl;
  
	float res;
	
	char c;
	string s = "";
	FILE *file;
	file = fopen(fileName.c_str(), "r");
	if (file) 
	{
		while ((c = getc(file)) != EOF)
		{
			if (c == 45 || c == 46 || (c >= 48 && c < 58))
			{
				s.push_back(c);
			}
			else
			{
				res = atof(s.c_str());
				cout << "float:" << res << endl;
				s = "";
				break;
			}
		}

		fclose(file);
	}
	else
	{
		cout << fileName << " - cannot open" << endl;
		return res;
	}

	return res;
}

#endif