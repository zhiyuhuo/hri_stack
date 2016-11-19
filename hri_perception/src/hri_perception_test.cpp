#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <pthread.h>

#include "std_msgs/String.h"
#include <std_msgs/Float64.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/PointCloud2.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "hri_perception/Perception/SEEntity.h"


using namespace std;

map<string, FurnitureDetector> detectorSet;
int RecognizeOneSample();
int TestAparmentData();
FurnitureDetector ReadOneFurnitureDetector(string rootDir, string name);
int ReadDetectors(string rootDir);
vector<vector<float> > LoadVectorVectorFloat(string fileName);
vector<float> LoadVectorFloat(string fileName);
float LoadFloat(string fileName);

int main(int argc, char **argv)
{	
	// PreProcessing
	printf("start perception...\n");
	string rootDir = "/home/hri/hri_DATA";
	ReadDetectors(rootDir);

	  
	TestAparmentData();
	
	
	return 0;
}

FurnitureDetector ReadOneFurnitureDetector(string rootDir, string name)
{
	FurnitureDetector res;
	res.name = name;
	res.sv = LoadVectorVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/supportvectors.txt");
	res.a = LoadVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/alpha.txt");
	res.b = LoadFloat(rootDir + "/FurnitureModelsForRos/" + name + "/bias.txt");
	res.l = LoadVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/group.txt");
	res.sf = LoadVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/scale.txt");
	res.sh = LoadVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/shift.txt");
	res.sigma = LoadFloat(rootDir + "/FurnitureModelsForRos/" + name + "/sigma.txt");
	res.anglew = LoadVectorFloat(rootDir + "/FurnitureModelsForRos/" + name + "/anglew.txt");
	return res;
}

int ReadDetectors(string rootDir)
{	
	int res = 0;
	
// 	FurnitureDetector fd1;
// 	fd1 = ReadOneFurnitureDetector(rootDir, "table_low");
// 	detectorSet["table_low"] = fd1;
// 	FurnitureDetector fd2;
// 	fd2 = ReadOneFurnitureDetector(rootDir, "table_high");
// 	detectorSet["table_high"] = fd2;
	FurnitureDetector fd3;
	fd3 = ReadOneFurnitureDetector(rootDir, "table");
	detectorSet["table"] = fd3;
	FurnitureDetector fd4;
	fd4 = ReadOneFurnitureDetector(rootDir, "chair");
	detectorSet["chair"] = fd4;
	FurnitureDetector fd5;
	fd5 = ReadOneFurnitureDetector(rootDir, "couch");
	detectorSet["couch"] = fd5;
	FurnitureDetector fd6;
	fd6 = ReadOneFurnitureDetector(rootDir, "bed");
	detectorSet["bed"] = fd6;
	
	return res;
}


vector<float> RecognizeOneSample(vector<int> index, vector<float> pt)
{
	SE s;
	s.ImportPt(index, pt);
	s.ProceedData(detectorSet);	
	cout << s.m_name << ", " << s.m_centroid[0] << " " << s.m_centroid[1] << ", " << s.m_dir << endl;
	return s.m_feature200;
}

int TestAparmentData()
{	
	int i = 1;
	int j = 0;
	char indexNameStr[50] = {};
	sprintf(indexNameStr, "/home/hri/HRI-Proj-Lib/Furniture/Raw/%d/index-%d-%d", i, i, j);
	char ptNameStr[50] = {};
	sprintf(ptNameStr, "/home/hri/HRI-Proj-Lib/Furniture/Raw/%d/pt-%d-%d", i, i, j);
	cout << i << "-" << j << " " << indexNameStr << endl;
	vector<int> index = ReadIntVectorFile10(indexNameStr);
	vector<float> pt = ReadFloatVectorFile10(ptNameStr);
	RecognizeOneSample(index, pt);

	int FNUM[8] = {32,24,36,24,36,32,24,24};
	vector<vector<float> > featureset; 
	for (int i = 2; i < 4; i++)
	{
		for (int j = 0; j < FNUM[i]; j++)
		{
			cout << i << " - " << j << endl;
			char indexNameStr[50] = {};
			sprintf(indexNameStr, "/home/hri/HRI-Proj-Lib/Furniture/Raw/%d/index-%d-%d", i, i, j);
			char ptNameStr[50] = {};
			sprintf(ptNameStr, "/home/hri/HRI-Proj-Lib/Furniture/Raw/%d/pt-%d-%d", i, i, j);
			cout << i << "-" << j << " " << ptNameStr << endl;
			vector<int> index = ReadIntVectorFile10(indexNameStr);
			vector<float> pt = ReadFloatVectorFile10(ptNameStr);
			vector<float> f = RecognizeOneSample(index, pt);
			featureset.push_back(f);
		}
	}	
	ofstream myfile;
	myfile.open ("/home/hri/featureset.out");
	for (size_t i = 0; i < featureset.size (); ++i)
	{
		vector<float> f = featureset[i];
		for (int j = 0; j < f.size(); j++)
		{
			myfile << f[j] << " "; 
		}
		myfile << endl;
	}
	myfile.close();
	
	return 0;
}

vector<vector<float> > LoadVectorVectorFloat(string fileName)
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

vector<float> LoadVectorFloat(string fileName)
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

float LoadFloat(string fileName)
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


