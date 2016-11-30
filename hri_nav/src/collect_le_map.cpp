#include "ros/ros.h"
#include <stdlib.h>
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
#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "hri_nav/Universal/Header.h"

#define GAZEBO

#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define IMG_SIZE 640*480
#define IMG_SIZE3 640*480*3

using namespace std;
using namespace cv;

//sensors
float curTiltAngle;
float tiltAngle;
float cameraHeight;
vector<float> rawPoints;
float posX, posY, posTheta;
bool ifGetPointCloud = false;
Mat imgRGB;
Mat imgDepth;
Mat imgMap;
string leName;
float mapHeight[400][400] = {};
geometry_msgs::Twist speedMsg;
string LEDir;


void ListenCallbackTilt(const std_msgs::Float64& msg);
void ListenCallbackPt2(const sensor_msgs::PointCloud2::ConstPtr& msg);
void ListenPoseCallback(const nav_msgs::OdometryConstPtr& msg);
vector<float> LocalToGlobal(float lx, float ly, float lth);
vector<int> CoordinateToPixel(float x, float y);
vector<float> PixelToCoordinate(int u, int v);
void DrawMap();
void KeyControl();
void SaveLE(vector<float> pts, string rootDir, string name);

int main(int argc, char **argv)
{
  
	if (argc > 1)
	{
		leName = string(argv[1]);
		cout << leName << endl;
	}
  
	// initialization global variables
	curTiltAngle = 0;
	tiltAngle = 0;
	cameraHeight = 1.105;
	imgRGB = Mat::zeros( IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
	imgDepth = Mat::zeros( IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	imgMap = Mat::zeros( 400, 400, CV_8UC3);
	vector<float> tp(IMG_SIZE3, 0);
	rawPoints = tp;
  
	printf("");
	ros::init(argc, argv, "hri_collect_le_map");
	ros::NodeHandle n;
	
	// topics
	
  	string platformStr;
	string ptTopicStr;
	string poseTopicStr;
	string cmdvelTopicStr;
	
	string key;
	if (n.searchParam("robot_platform", key))
	{
	    n.getParam(key, platformStr);
	}
	if (platformStr.compare("physical") == 0)
	{
	    ptTopicStr = "/camera/depth_registered/points";
	    poseTopicStr = "/pose";
	    cmdvelTopicStr = "/cmd_vel";
	}
	else
	{
	    ptTopicStr = "/camera/depth/points";
	    poseTopicStr = "/hri_robot/odom";
	    cmdvelTopicStr = "/hri_robot/cmd_vel";  
	}
	
	ros::Subscriber pt2Sub = n.subscribe(ptTopicStr.c_str(), 10, ListenCallbackPt2);
	
	ros::Subscriber pose2Sub = n.subscribe(poseTopicStr.c_str(), 10, ListenPoseCallback);

	ros::Subscriber tiltSub = n.subscribe("cur_tilt_angle", 100, ListenCallbackTilt);
	
	ros::Publisher tiltPub = n.advertise<std_msgs::Float64>("/tilt_angle", 100);
	
	ros::Publisher speedPub = n.advertise<geometry_msgs::Twist>(cmdvelTopicStr.c_str(), 100);
	
	// Define messages
	std_msgs::Float64 tiltMsg;
	
	//LE
	string worldNameStr;
	if (n.searchParam("world_name", key))
	{
	    n.getParam(key, worldNameStr);
	}
	
	//Get Le
	string LEDirStr("/home/hri/hri_DATA/LE_" + worldNameStr + "/");
	LEDir = LEDirStr;
	
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
	
	
	while(ros::ok() && (!ifGetPointCloud))
	{
		ros::spinOnce();
	}

	cout << "robot loop start..." << endl;
	while (ros::ok())
	{	
	  	KeyControl();
		imshow("Map", imgMap);
		imshow("Depth", imgDepth);
		waitKey(1);	
		
		speedPub.publish(speedMsg);
		tiltPub.publish(tiltMsg);
		
		ros::spinOnce();

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

		if (!isnan(x))
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

	rawPoints = xyzRaw;
	ifGetPointCloud = true;
}

void ListenPoseCallback(const nav_msgs::OdometryConstPtr& msg)
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

	//cout << "x: " << posX << ",	y: " << posY << ",		theta: " << posTheta * 180 / PI << endl;
}

vector<float> LocalToGlobal(float lx, float ly, float lth)
{
	vector<float> res(3, 0);
	float px = posX;
	float py = posY;
	float th = posTheta - PI / 2;
	float gx = cos(th) * lx + -sin(th) * ly + px;
	float gy = sin(th) * lx + cos(th) * ly + py;
	res[0] = gx;
	res[1] = gy;
	res[2] = lth + th;
	while (res[2] > 2 * PI)
	{
		res[2] -= 2 * PI;
	}
	while (res[2] < 0)
	{
		res[2] += 2 * PI;
	}
	
	return res;
}

vector<float> PixelToCoordinate(int u, int v)
{
	vector<float> res(2, 0);
	res[0] = (u - 200.0) * 0.1;
	res[1] = (v - 200.0) * 0.1;
	return res;
}

vector<int> CoordinateToPixel(float x, float y)
{
	vector<int> res(2, 0);
	res[0] = (int)(x / 0.1 + 200);
	res[1] = (int)(y / 0.1 + 200);
	return res;	
}

void DrawMap()
{
  	rawPoints = Transformation(rawPoints, cameraHeight, curTiltAngle);
	vector<float> pts2 = rawPoints;
	vector<int> p;
	for (int i = 0; i < pts2.size() / 3; i++)
	{
		if (!isnan(pts2[3*i]) && pts2[3*i+2] > 0.2)
		{
			vector<float> gp = LocalToGlobal(pts2[3*i], pts2[3*i+1], 0);
			p = CoordinateToPixel(gp[0], gp[1]);		
			
			if (pts2[3*i+2] > mapHeight[p[1]][p[0]])
			{
				mapHeight[p[1]][p[0]] = pts2[3*i+2];
			}
		}
	}
	
	for (int u = 0; u < 400; u++)
	{
		for (int v = 0; v < 400; v++)
		{
			unsigned int c = u + v * 400;
			if (mapHeight[v][u] > 1.25 && mapHeight[v][u] < 2.25)
			{
				imgMap.data[3*c] = 255;
				imgMap.data[3*c+1] = 255;
				imgMap.data[3*c+2] = 255;				
			}
			else if (mapHeight[v][u] <= 1.0 && mapHeight[v][u] > 0.2)
			{
				imgMap.data[3*c] = 0;
				imgMap.data[3*c+1] = 255;
				imgMap.data[3*c+2] = 0;				
			}
		}
	}
}

void KeyControl()
{
	char c = waitKey(30);
	switch (c)
	{
		case 'w':
		{
			speedMsg.linear.x = 0.5;
			speedMsg.angular.z = 0.0;
			break;
		}
		
		case 'x':
		{
			speedMsg.linear.x = -0.5;
			speedMsg.angular.z = 0.0;
			break;
		}
		
		case 'a':
		{
			if (speedMsg.linear.x >= 0)
			{
				speedMsg.angular.z = 0.5;
			}
			else
			{
				speedMsg.angular.z = -0.5;
			}
			#ifdef GAZEBO
			speedMsg.angular.z = -speedMsg.angular.z;
			#endif
			break;
		}
		
		case 'd':
		{
			if (speedMsg.linear.x >= 0)
			{
				speedMsg.angular.z = -0.5;
			}
			else
			{
				speedMsg.angular.z = 0.5;
			}
			#ifdef GAZEBO
			speedMsg.angular.z = -speedMsg.angular.z;
			#endif
			break;
		}
		
		case 's':
		{
			speedMsg.linear.x = 0;
			speedMsg.angular.z = 0;
			break;
		}
		
		case 'r':
		{
		  	cout << "start to map: " << endl;
		  	DrawMap();
			break;
		}
		
		case 'y':
		{
			SaveLE(rawPoints, LEDir.c_str(), leName);
			exit(0);
			break;
		}
		
		default:
		{
			break;
		}
	}
}

void SaveLE(vector<float> pts, string rootDir, string name)
{
	ofstream myfile;
	string filename = rootDir + name + ".le";
	cout << "Saving to: " << filename << endl;
	myfile.open (filename.c_str());

	for (int u = 0; u < 400; u++)
	{
		for (int v = 0; v < 400; v++)
		{
			unsigned int c = u + v * 400;
			if (imgMap.data[3*c] == 255 
			  && imgMap.data[3*c+1] == 255
			  && imgMap.data[3*c+2] == 255	)
			{
				vector<float> p = PixelToCoordinate(u, v);
				myfile << p[0] << " " << p[1] << endl;
			}
		}
	}
	
	cout << "Writing this to a file.\n";
	myfile.close();
}

















