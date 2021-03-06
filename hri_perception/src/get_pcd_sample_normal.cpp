#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "hri_perception/Universal/Header.h"

using namespace std;

pcl::PointCloud<pcl::Normal> get_normal_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{	

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (0.03);
	ne.compute (*normals);	
	
	return *normals;
}

void save_pc_norm(string fileName, pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> normals)
{
	ofstream myfile;
	myfile.open (fileName.c_str());
	for (size_t i = 0; i < cloud.points.size (); ++i)
	{
		if (!isnan(normals.points[i].normal_x) && !isnan(normals.points[i].normal_y) && !isnan(normals.points[i].normal_z))
		{
			myfile << cloud.points[i].x << " " 
				<< cloud.points[i].y << " " 
				<< cloud.points[i].z << " " 
				<< normals.points[i].normal_x << " " 
				<< normals.points[i].normal_y << " " 
				<< normals.points[i].normal_z << " " 
				<< "\n";	
		}
	}
	myfile.close();
}

void get_apartment_dataset_feature()
{
	int FNUM[8] = {32,24,36,24,36,32,24,24};
	string CAT[8] = {"table", "chair", "table", "chair", "table", "table", "couch", "bed"};

	for (int instance = 0; instance < 8; instance++)
	{
		//for (int sample = 0; sample < 1; sample++)
		for (int sample = 0; sample < FNUM[instance]; sample++)
		{	
			//read PCD
			char pcdNameStr[100] = {};
			sprintf(pcdNameStr, "/home/hri/Samples/PCD/hri_apartment/%d/pcd-%d-%d.pcd", instance, instance, sample);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::io::loadPCDFile<pcl::PointXYZ> (pcdNameStr, *cloudin);
			
			//voxelize pc
			pcl::VoxelGrid<pcl::PointXYZ> sor;
			sor.setInputCloud (cloudin);
			sor.setLeafSize (0.02f, 0.02f, 0.02f);
			sor.filter (*cloudin);
			
			//get normal feature
			pcl::PointCloud<pcl::PointXYZ> cloud = *cloudin;
			pcl::PointCloud<pcl::Normal> normals = get_normal_feature(cloudin);
			
			//save normals
			char pcnormNameStr[100] = {};
			sprintf(pcnormNameStr, "/home/hri/Samples/Normal/hri_apartment/%d/pcnorm-%d-%d.txt", instance, instance, sample);
			string pcnormName(pcnormNameStr);
			cout << "saving " << pcnormName << endl;
			save_pc_norm(pcnormName, cloud, normals);
			cout << "save done. " << endl;
		}
	}	
}

void get_rgbd_scene_dataset_feature()
{
	string CAT[4] = {"coffee_table", "office_chair", "sofa", "table"};
	for (int n = 0; n < 4; n++)
	{
	  	vector<string> fileList;
		string catogeryDir = "/home/hri/Samples/PCD/rgbd_scene/" + CAT[n];
		DIR *pDIR;
		struct dirent *entry;
		if( pDIR=opendir(catogeryDir.c_str()) )
		{
			while(entry = readdir(pDIR))
			{
				if( strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 )
				{
					string str(entry->d_name);
					if (str.find(".pcd") != string::npos)
					{
						fileList.push_back(str);
						cout << str << "\n";
					}
				}
			}
			closedir(pDIR);
		}
		
		for (int i = 0; i < fileList.size(); i++)
		{
			//read PCD
			char pcdNameStr[100] = {};
			sprintf(pcdNameStr, "%s/%s", catogeryDir.c_str(), fileList[i].c_str());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::io::loadPCDFile<pcl::PointXYZ> (pcdNameStr, *cloudin);
				
			//voxelize pc
			pcl::VoxelGrid<pcl::PointXYZ> sor;
			sor.setInputCloud (cloudin);
			sor.setLeafSize (0.02f, 0.02f, 0.02f);
			sor.filter (*cloudin);
				
			//get normal feature
			pcl::PointCloud<pcl::PointXYZ> cloud = *cloudin;
			pcl::PointCloud<pcl::Normal> normals = get_normal_feature(cloudin);
				
			//save normals
			string txtFileName((fileList[i]).begin(), (fileList[i]).end()-4);
			char pcnormNameStr[100] = {};
			sprintf(pcnormNameStr, "/home/hri/Samples/Normal/rgbd_scene/%s/pcnorm-%s.txt", CAT[n].c_str(), txtFileName.c_str());
			string pcnormName(pcnormNameStr);
			cout << "saving " << pcnormName << endl;
			save_pc_norm(pcnormName, cloud, normals);
			cout << "save done. " << endl;	
		}
	}
}

int main()
{
	get_apartment_dataset_feature();
	get_rgbd_scene_dataset_feature();
	
	return 0;
}
