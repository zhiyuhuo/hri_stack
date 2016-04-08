#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#define PI 3.1416

using namespace std;

float x, y, qx, qy, qz, qw, theta;
float curTiltAngle;
float cameraHeight;

void poseCallback(const nav_msgs::OdometryConstPtr& msg){

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
}

void ListenCallbackTilt(const std_msgs::Float64& msg)
{
	//printf("tilt: msg: %f\n", msg.data);
	curTiltAngle = (float)msg.data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hri_tf_broadcaster");
    
    curTiltAngle = 0;
    cameraHeight = 1.10;

    ros::NodeHandle node;
    ros::Subscriber poseSub = node.subscribe("/hri_robot/odom", 10, &poseCallback);
    ros::Subscriber tiltSub = node.subscribe("cur_tilt_angle", 1000, &ListenCallbackTilt);

    while(ros::ok())
    {
        //build the tf for robot
        static tf::TransformBroadcaster brRobot;
        tf::Transform transformRobot;
        transformRobot.setOrigin( tf::Vector3(x, y, 0.0) );
        tf::Quaternion qRobot;
        qRobot.setRPY(0, 0, theta);
        transformRobot.setRotation(qRobot);
        brRobot.sendTransform(tf::StampedTransform(transformRobot, ros::Time::now(), "map", "hri_robot/odom"));
        
        //build the tf for kinect
        static tf::TransformBroadcaster brTilt;
        tf::Transform transformTilt;
        transformTilt.setOrigin( tf::Vector3(0.0, cameraHeight, 0.0) );
        tf::Quaternion qTilt;
        qTilt.setRPY(0, 0, PI/2);
        transformTilt.setRotation(qTilt);
        brTilt.sendTransform(tf::StampedTransform(transformTilt, ros::Time::now(), "hri_robot/odom", "openni_camera_link"));	
    }
    return 0;
};
