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
#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf2_msgs/TFMessage.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "hri_perception/Universal/Header.h"

#define GAZEBO

#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define IMG_SIZE 640*480
#define IMG_SIZE3 640*480*3

#define G_X 20
#define G_Y 20

using namespace std;

int Process(const vector<float>& pts, cv::Mat Map, cv::Mat labelMap);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "build_semantic_map");
	ros::NodeHandle n;
    
    string pcdName = "/home/hri/hri_DATA/test/" + string(argv[1]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ> (pcdName.c_str(), *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.02f, 0.02f, 0.02f);
    sor.filter (*cloudFiltered);
	
    vector<float> pts(cloudFiltered->points.size()*3, 0);
    for (int i = 0; i < cloudFiltered->points.size(); i++)
    {
        pts[3*i] = cloudFiltered->points[i].x;
        pts[3*i+1] = cloudFiltered->points[i].y;
        pts[3*i+2] = cloudFiltered->points[i].z;
    }
    
    int W = G_X / LOCALMAP_RESOLUTION;
    int H = G_Y / LOCALMAP_RESOLUTION;
    
    cv::Mat Map = cv::Mat::zeros(H, W, CV_8UC1);
	cv::Mat labelMap = cv::Mat::zeros(H, W, CV_8UC1);
    Process(pts, Map, labelMap);
    
    while (ros::ok())
    {
        cv::imshow("map", Map);
        cv::waitKey(1);
    }
    
	return 0;
}

int Process(const vector<float>& pts, cv::Mat Map, cv::Mat labelMap)
{	
	BuildZeroCentroidLocalMap(pts, Map, G_X, G_Y);
	BwLabel(Map, labelMap);
	SetBwMapBackgroundZero(labelMap);
    
    return 0;
}

