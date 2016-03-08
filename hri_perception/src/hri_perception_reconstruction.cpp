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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf2_msgs/TFMessage.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include "hri_perception/Env.h"

#include "hri_perception/Universal/Header.h"
#include "hri_perception/Perception/Header.h"

#define GAZEBO

#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define IMG_SIZE 640*480
#define IMG_SIZE3 640*480*3

using namespace std;


//sensors
float curTiltAngle;
float tiltAngle;
float cameraHeight;
vector<float> rawPoints;
float posX, posY, posTheta;
bool ifGetPointCloud = false;
cv::Mat imgRGB;
cv::Mat imgDepth;

void ListenCallbackTilt(const std_msgs::Float64& msg);
void ListenCallbackPt2(const sensor_msgs::PointCloud2::ConstPtr& msg);
void ListenCallbackPose(const nav_msgs::OdometryConstPtr& msg);
vector<float> LocalToGlobal(vector<float> lp);


int main(int argc, char **argv)
{
	// initialization global variables
	curTiltAngle = -20;
	tiltAngle = -20;
	cameraHeight = 1.105;
	imgRGB = cv::Mat::zeros( IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
	imgDepth = cv::Mat::zeros( IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	vector<float> tp(IMG_SIZE3, 0);
	rawPoints = tp;
  
	//printf("");
	ros::init(argc, argv, "hri_perception");
	ros::NodeHandle n;
	
	// subscribed topics
	
	#ifdef GAZEBO
	ros::Subscriber pt2Sub = n.subscribe("/camera/depth/points", 10, ListenCallbackPt2);
	#else
	ros::Subscriber pt2Sub = n.subscribe("/camera/depth_registered/points", 10, ListenCallbackPt2);
	#endif
	
	#ifdef GAZEBO
	ros::Subscriber tiltSub = n.subscribe("cur_tilt_angle", 1000, ListenCallbackTilt);
	#else
	ros::Subscriber tiltSub = n.subscribe("cur_tilt_angle", 1000, ListenCallbackTilt);
	#endif
	
	#ifdef GAZEBO
	ros::Subscriber pose2Sub = n.subscribe("/hri_robot/odom", 10, ListenCallbackPose);
	#else
	ros::Subscriber pose2Sub = n.subscribe("/pose", 10, ListenCallbackPose);
	#endif
	
	// published tilt topic
	ros::Publisher envPub = n.advertise<hri_perception::Env>("/env", 10);
	
	// published env model topic
	ros::Publisher tiltPub = n.advertise<std_msgs::Float64>("/tilt_angle", 100);
	
	// Define messages
	std_msgs::Float64 tiltMsg; 
    
    //slam data
    vector<float> scenePoints;
    float resolution = 32.0f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPast (new pcl::PointCloud<pcl::PointXYZ> );
    //octree 
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
	
	
	// PreProcessing
	printf("start robot...\n");
	for (int i = 0; i < 5; i++) // preprocess to fix the tilt
	{
		struct timespec timeOut,remains;
		timeOut.tv_sec = 0;
		timeOut.tv_nsec = 500000000; // 50 milliseconds 
		nanosleep(&timeOut, &remains);
		printf("%d\n",4-i);
		tiltMsg.data = tiltAngle;
		tiltPub.publish(tiltMsg);	
	}
	
	
		
	ros::Rate loopRate(30);
	while(ros::ok() && (!ifGetPointCloud))
	{
		ros::spinOnce();
	}

	cout << "robot loop start..." << endl;
	while (ros::ok())
	{	
		vector<float> pts = Transformation(rawPoints, cameraHeight, curTiltAngle);
        vector<float> gpts = LocalToGlobal(pts);
        
        //octree find spatial change
        octree.setInputCloud (cloudPast);
        octree.addPointsFromInputCloud ();
        octree.switchBuffers ();
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNow (new pcl::PointCloud<pcl::PointXYZ> );
        cloudNow->width = gpts.size() / 3;
        cloudNow->height = 1;
        cloudNow->points.resize (cloudNow->width * cloudNow->height);
        for (size_t i = 0; i < cloudNow->points.size (); ++i)
        {
            cloudNow->points[i].x = gpts[3*i];
            cloudNow->points[i].y = gpts[3*i+1];
            cloudNow->points[i].z = gpts[3*i+2];
        }
        octree.setInputCloud (cloudNow);
        octree.addPointsFromInputCloud ();
        
        std::vector<int> newPointIdxVector;
        octree.getPointIndicesFromNewVoxels (newPointIdxVector);

		
        
		ros::spinOnce();
		loopRate.sleep();
	}
	  
	return 0;
}

void ListenCallbackTilt(const std_msgs::Float64& msg)
{
	//printf("tilt: msg: %f\n", msg.data);
	curTiltAngle = (float)msg.data;
}

void ListenCallbackPt2(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//printf("%d %d %d %d\n", msg->point_step, msg->row_step, msg->width, msg->height);
	int step = (int)msg->point_step;
	vector<float> xyzRaw(IMG_SIZE3, 0);
	for (int i = 0; i < IMG_SIZE; i++)
	{
		float x, y, z;
		memcpy(&x, &msg->data[step*i], 4);
		memcpy(&y, &msg->data[step*i+4], 4);
		memcpy(&z, &msg->data[step*i+8], 4);
		xyzRaw[3*i] = x;
		xyzRaw[3*i+1] = y;
		xyzRaw[3*i+2] = z;

		if (x == x || y == y || z == z)
		{
			imgRGB.data[3*i] = msg->data[step*i+16];
			imgRGB.data[3*i+1] = msg->data[step*i+17];
			imgRGB.data[3*i+2] = msg->data[step*i+18];
			unsigned char dp = (unsigned char)((sqrt(x*x + y*y + z*z) / 8) * 256);
			//cout << x << " " << y << " " << z << " " << (sqrt(x*x + y*y + z*z) / 8) * 256 << " " << dp << endl;
			if (dp > 255)
			{
			      dp = 0;
			}
			else
			{
			      imgDepth.data[i] = dp;
			}
		}
		else
		{
			imgRGB.data[3*i] = 0;
			imgRGB.data[3*i+1] = 0;
			imgRGB.data[3*i+2] = 0;
			imgDepth.data[i] = 0;
		}

	}
	
	cv::imshow("Depth", imgDepth);
	cv::waitKey(1);	

	rawPoints = xyzRaw;
	ifGetPointCloud = true;
}

void ListenCallbackPose(const nav_msgs::OdometryConstPtr& msg)
{
	float x, y, qx, qy, qz, qw, theta;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;
	float rotz = atan2(2*qw*qz, 1-2*qz*qz);
	theta = rotz;
	
	while (theta < 0)
	{
		theta += 2*PI;
	}
	
	while (theta > 2*PI)
	{
		theta -= 2*PI;
	}

	posX = x;
	posY = y;
	posTheta = theta;	
	
// 	cout << "x: " << m_posRobot.GetX() << ",	y: " << m_posRobot.GetY() << ",		theta: " << m_theta << endl;
}

vector<float> LocalToGlobal(vector<float> lp)
{
    int L = lp.size() / 3;
	vector<float> res(L*3, 0);
    for (int i = 0; i < L; i++)
    {
        res[3*i] = cos(th) * lp[3*i] + -sin(th) * lp[3*i+1] + posX;
        res[3*i+1] = sin(th) * lp[3*i] + cos(th) * lp[3*i+1] + posY;
        res[3*i+2] = lp[3*i+2];
    }
	
	return res;
}