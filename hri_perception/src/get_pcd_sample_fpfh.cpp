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
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>

#include "hri_perception/Universal/Header.h"

using namespace std;

void save_pc_fpfh(string fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs)
{
	ofstream myfile;
	myfile.open (fileName.c_str());
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{	
		if (!isnan(cloud->points[i].x) && !isnan(cloud->points[i].y) && !isnan(cloud->points[i].z))
		{
			myfile << cloud->points[i].x << " " 
				<< cloud->points[i].y << " " 
				<< cloud->points[i].z << " ";	
			
			for (int j = 0; j < 33; j++)
			{
				myfile << fpfhs->points[i].histogram[j] << " ";
			}
			myfile << endl;
		}
	}
	myfile.close();
}

void build_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature)
{
	//voxelize pc
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.02f, 0.02f, 0.02f);
	sor.filter (*cloud);
			
	//get normal feature
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (0.03);
	ne.compute (*normals);	
			
	//get fpfh feature
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud);
	fpfh.setInputNormals (normals);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
	fpfh.setSearchMethod (tree2);
	fpfh.setRadiusSearch (0.05);
	fpfh.compute (*feature);	
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
			//read PCD get point cloud
			char pcdNameStr[100] = {};
			sprintf(pcdNameStr, "/home/hri/Samples/PCD/hri_apartment/%d/pcd-%d-%d.pcd", instance, instance, sample);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::io::loadPCDFile<pcl::PointXYZ> (pcdNameStr, *cloud);
			
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature (new pcl::PointCloud<pcl::FPFHSignature33>);
			build_fpfh_feature(cloud, feature);
			
			char pcfpfhNameStr[100] = {};
			sprintf(pcfpfhNameStr, "/home/hri/Samples/FPFH/hri_apartment/%d/pcfpfh-%d-%d.txt", instance, instance, sample);
			string pcfpfsName(pcfpfhNameStr);
			cout << "saving " << pcfpfsName << endl;
			save_pc_fpfh(pcfpfsName, cloud, feature);
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
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::io::loadPCDFile<pcl::PointXYZ> (pcdNameStr, *cloud);
				
			//get fpfh feature
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature (new pcl::PointCloud<pcl::FPFHSignature33>);
			build_fpfh_feature(cloud, feature);
				
			//save normals
			string txtFileName((fileList[i]).begin(), (fileList[i]).end()-4);
			char pcfpfhNameStr[100] = {};
			sprintf(pcfpfhNameStr, "/home/hri/Samples/FPFH/rgbd_scene/%s/pcfpfh-%s.txt", CAT[n].c_str(), txtFileName.c_str());
			string pcfpfsName(pcfpfhNameStr);
			cout << "saving " << pcfpfsName << endl;
			save_pc_fpfh(pcfpfsName, cloud, feature);
			cout << "save done. " << endl;
		}
	}
}

int main()
{
	//get_apartment_dataset_feature();
	get_rgbd_scene_dataset_feature();
	return 0;
}
