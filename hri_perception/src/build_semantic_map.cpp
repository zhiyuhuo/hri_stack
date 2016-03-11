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


int main(int argc, char **argv)
{

		
	_perception.ImportKinectData(imgRGB, pts);
	_perception.Process();

	  
	return 0;
}

