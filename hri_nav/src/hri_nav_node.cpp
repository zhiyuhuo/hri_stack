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
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "p2os_msgs/MotorState.h"
#include "p2os_msgs/SonarArray.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf2_msgs/TFMessage.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define GAZEBO

#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define IMG_SIZE 640*480
#define IMG_SIZE3 640*480*3
#define PI 3.1416

using namespace std;
using namespace cv;

//sensors
float curTiltAngle;
float tiltAngle;
float cameraHeight;
vector<float> rawPoints;
float posX, posY, posTheta;
geometry_msgs::Twist speedMsg;
bool ifGetPointCloud = false;
Mat imgRGB;
Mat imgDepth;

void ListenCallbackPose(const nav_msgs::OdometryConstPtr& msg);
void ListenCallbackTilt(const std_msgs::Float64& msg);
void ListenCallbackPt2(const sensor_msgs::PointCloud2::ConstPtr& msg);
sensor_msgs::LaserScan BuildLaserMsg(vector<float> pt);
vector<float> Transformation(vector<float> src, float height, float angle);


int main(int argc, char **argv)
{
	// initialization global variables
	curTiltAngle = 0;
	tiltAngle = 0;
	cameraHeight = 1.105;
	imgRGB = Mat::zeros( IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
	imgDepth = Mat::zeros( IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	vector<float> tp(IMG_SIZE3, 0);
	rawPoints = tp;
	posX = 0;
	posY = 0;
	posTheta = PI / 2;
  
	printf("");
	ros::init(argc, argv, "hri_nav");
	ros::NodeHandle n;
	
	// subscribed topics

	#ifdef GAZEBO
	ros::Subscriber poseSub = n.subscribe("hri_robot/odom", 1000, ListenCallbackPose);
	#else
	ros::Subscriber poseSub = n.subscribe("/pose", 1000, ListenCallbackPose);
	#endif
	
	#ifdef GAZEBO
	ros::Subscriber pt2Sub = n.subscribe("/camera/depth/points", 10, ListenCallbackPt2);
	#else
	ros::Subscriber pt2Sub = n.subscribe("/camera/depth_registered/points", 10, ListenCallbackPt2);
	#endif
	
	ros::Subscriber tiltSub = n.subscribe("cur_tilt_angle", 1000, ListenCallbackTilt);
	
	// published topics
	ros::Publisher motorstatePub = n.advertise<p2os_msgs::MotorState>("/cmd_motor_state", 10);
	
	ros::Publisher tiltPub = n.advertise<std_msgs::Float64>("/tilt_angle", 100);
	
	#ifdef GAZEBO
	ros::Publisher speedPub = n.advertise<geometry_msgs::Twist>("hri_robot/cmd_vel", 10);
	#else
	ros::Publisher speedPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	#endif
	
	ros::Publisher scanPub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
	
	// Define messages
	std_msgs::Float64 tiltMsg;
	p2os_msgs::MotorState motorMsg;
	geometry_msgs::Twist speedMsg;
	sensor_msgs::LaserScan scanMsg;
	
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
		motorMsg.state = (int)1;
		motorstatePub.publish(motorMsg);
	}
	
	
		
	ros::Rate loopRate(30);
	while(ros::ok() && (!ifGetPointCloud))
	{
		ros::spinOnce();
	}

	cout << "robot loop start..." << endl;
	while (ros::ok())
	{	
		vector<float> pts = Transformation(rawPoints, cameraHeight, -curTiltAngle);
		scanMsg = BuildLaserMsg(pts);
		scanPub.publish(scanMsg);
		
		
		speedPub.publish(speedMsg);		
		imshow("Depth", imgDepth);
		waitKey(1);	
		ros::spinOnce();
		loopRate.sleep();
	}
	  
	return 0;
}

void ListenCallbackPose(const nav_msgs::OdometryConstPtr& msg)
{
	float x, y, qx, qy, qz, qw, th, theta;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;
	float rotz = atan2(2*qw*qz, 1-2*qz*qz);
	th = rotz;

	#ifdef GAZEBO
	if (th >=  - PI/2)
	{
		theta = th + PI / 2;
	}
	else 
	{
		theta = th + 2.5 * PI;
	}
	#else
	theta = th;
	#endif
	
	//cout << x << ", " << y << ", " << th << endl;

	posX = -y;
	posY = x;
	posTheta = theta;
}

void ListenCallbackTilt(const std_msgs::Float64& msg)
{
	//printf("tilt: msg: %f\n", msg.data);
	curTiltAngle = msg.data;
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

	rawPoints = xyzRaw;
	ifGetPointCloud = true;
}

sensor_msgs::LaserScan BuildLaserMsg(vector<float> pt)
{
  	sensor_msgs::LaserScan res;
	unsigned int numReadings = 60;
	float laser_frequency = 40;
	vector<float> ranges(numReadings, 10);
	float x,y,z;
	float d = 0;
	float angle = 0;
	for (int i = 0; i < pt.size()/3; i++)
	{
		x = pt[3*i];
		y = pt[3*i+1];
		z = pt[3*i+2];
		//cout << x << " " << y << " " << z << endl;	
	  	if (x == x && y == y && z == z)
		{
			if (z > 1.5 && z < 2)
			{
				d = sqrt(x*x+y*y);
				angle = atan2(x, y);
				int ag = (int)((angle - (-PI/6)) / (PI/180));
				//cout << ag << endl;
				if (ag >=0 && ag <numReadings)
				{
					if (d < ranges[ag])
					{
						ranges[ag] = d;
					}
				}
			}
		}
	}

	//generate some fake data for our laser scan
	ros::Time scanTime = ros::Time::now();

	//populate the LaserScan message
	res.header.stamp = scanTime;
	res.header.frame_id = "laser_frame";
	res.angle_min = -PI/6;
	res.angle_max =  PI/6;
	res.angle_increment = PI / numReadings;
	res.time_increment = (1 / laser_frequency) / (numReadings);
	res.range_min = 0.1;
	res.range_max = 8.0;

	res.ranges = ranges;
	
	return res;
}

vector<float> Transformation(vector<float> src, float height, float angle)
{
	vector<float> dst(src.size(), 0);
	angle = angle * PI / 180;

	for(int i = 0; i < src.size()/3; i++)
	{
		float x0, y0, z0;
		float x, y, z;

		x0 = src[3*i];
		y0 = src[3*i+2];
		z0 = -src[3*i+1];

		x = x0;
		y = cos(angle)*y0 + sin(angle)*z0;
		z = -sin(angle)*y0 + cos(angle)*z0 + height;

		dst[3*i] = x;
		dst[3*i+1] = y;
		dst[3*i+2] = z;
	}
	return dst;

}