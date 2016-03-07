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

int main()
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
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
			fpfh.setRadiusSearch (0.05);
			fpfh.compute (*fpfhs);
			
			//save normals
			char pcnormNameStr[100] = {};
			sprintf(pcnormNameStr, "/home/hri/Samples/FPFH/hri_apartment/%d/pcfpfh-%d-%d.txt", instance, instance, sample);
			string pcnormName(pcnormNameStr);
			cout << "saving " << pcnormName << endl;
			//save_pc_fpfh(pcnormName, cloud, normals);
			cout << "save done. " << endl;
		}
	}
	
	
	return 0;
}
