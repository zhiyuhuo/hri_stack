#include "ros/ros.h"
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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/PointCloud2.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "hri_perception/Perception/GOEntity.h"


using namespace std;
using namespace cv;

map<string, vector<float> > voxelDetector;
int RecognizeOneSample();
int TestAparmentData();

int main(int argc, char **argv)
{	
	// PreProcessing
	printf("start perception...\n");
	string rootDir = "/home/hri/hri_DATA";
	
	vector<float> f;
	f = ReadFloatVectorFile(rootDir + "/V/table-low.v");
	voxelDetector["table-low"] = f;
	f = ReadFloatVectorFile(rootDir + "/V/table-high.v");
	voxelDetector["table-high"] = f;
	f = ReadFloatVectorFile(rootDir + "/V/chair-front.v");
	voxelDetector["chair-front"] = f;
	f = ReadFloatVectorFile(rootDir + "/V/chair-back.v");
	voxelDetector["chair-back"] = f;
	f = ReadFloatVectorFile(rootDir + "/V/bed.v");
	voxelDetector["bed"] = f;
	f = ReadFloatVectorFile(rootDir + "/V/couch-front.v");
	voxelDetector["couch-front"] = f;
	  
	TestAparmentData();
	
	
	return 0;
}

int RecognizeOneSample(vector<int> index, vector<float> pt)
{
	GO go;
	go.ImportPt(index, pt);
	go.ProceedData(voxelDetector);	
	cout << go.m_name << ", " << go.m_dir << ", " << index.size() << ", " << pt.size() << ", " << go.m_centroid[0] << " " << go.m_centroid[1] << endl;
	return 0;
}

int TestAparmentData()
{
	int FNUM[8] = {32,24,36,24,36,32,24,24};
	vector<pair<int, string> >  lib;

	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < FNUM[i]; j++)
		{
			char indexNameStr[50] = {};
			sprintf(indexNameStr, "/home/hri/HRI-Proj-Lib/Furniture/Raw/%d/index-%d-%d", i, i, j);
			char ptNameStr[50] = {};
			sprintf(ptNameStr, "/home/hri/HRI-Proj-Lib/Furniture/Raw/%d/pt-%d-%d", i, i, j);
			cout << i << "-" << j << " " << indexNameStr << endl;
			vector<int> index = ReadIntVectorFile10(indexNameStr);
			vector<float> pt = ReadFloatVectorFile10(ptNameStr);
			RecognizeOneSample(index, pt);
		}
	}
	
	
	return 0;
}