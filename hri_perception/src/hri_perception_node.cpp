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

Perception _perception;

void ListenCallbackTilt(const std_msgs::Float64& msg);
void ListenCallbackPt2(const sensor_msgs::PointCloud2::ConstPtr& msg);
hri_perception::Env BuildEnvMsg();


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
	
	// published tilt topic
	ros::Publisher envPub = n.advertise<hri_perception::Env>("/env", 10);
	
	// published env model topic
	ros::Publisher tiltPub = n.advertise<std_msgs::Float64>("/tilt_angle", 100);
	
	// Define messages
	std_msgs::Float64 tiltMsg;
	hri_perception::Env envMsg;
	
	string furnitureDctFolder = "/home/hri/hri_DATA";
	_perception.ReadDetectors(furnitureDctFolder);
	
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
		
		_perception.ImportKinectData(imgRGB, pts);
		_perception.Process();
	
		envMsg = BuildEnvMsg();
		envPub.publish(envMsg);
		
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

hri_perception::Env BuildEnvMsg()
{
	hri_perception::Env env;
  
	int L = _perception.m_GOList.size();
	
	if (L >= 1)
	{
		env.name0 = _perception.m_GOList[0].name;
		env.dir0 = _perception.m_GOList[0].dir;
		env.vec0 = _perception.m_GOList[0].vec;
	}
	
	if (L >= 2)
	{
		env.name1 = _perception.m_GOList[1].name;
		env.dir1 = _perception.m_GOList[1].dir;
		env.vec1 = _perception.m_GOList[1].vec;
	}
	
	if (L >= 3)
	{
		env.name2 = _perception.m_GOList[2].name;
		env.dir2 = _perception.m_GOList[2].dir;
		env.vec2 = _perception.m_GOList[2].vec;
	}
	
	if (L >= 4)
	{
		env.name3 = _perception.m_GOList[3].name;
		env.dir3 = _perception.m_GOList[3].dir;
		env.vec3 = _perception.m_GOList[3].vec;
	}
	
	if (L >= 5)
	{
		env.name4 = _perception.m_GOList[4].name;
		env.dir4 = _perception.m_GOList[4].dir;
		env.vec4 = _perception.m_GOList[4].vec;
	}
	
	if (L >= 6)
	{
		env.name5 = _perception.m_GOList[5].name;
		env.dir5 = _perception.m_GOList[5].dir;
		env.vec5 = _perception.m_GOList[5].vec;
	}
	
	return env;
}