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
    //cout << "receive pose\n";
}

void ListenCallbackTilt(const std_msgs::Float64& msg)
{
	//printf("tilt: msg: %f\n", msg.data);
	curTiltAngle = (float)msg.data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hri_tf_broadcaster");
    
    curTiltAngle = 0;
    cameraHeight = 1.05;

    ros::NodeHandle node;
    ros::Subscriber poseSub = node.subscribe("/hri_robot/odom", 10, &poseCallback);
    ros::Subscriber tiltSub = node.subscribe("cur_tilt_angle", 1000, &ListenCallbackTilt);

    ros::Rate loop_rate(10);
    float curTiltAngleRad;
    while(ros::ok())
    {
        curTiltAngleRad = curTiltAngle * PI / 180;

        //build the tf for kinect
        static tf::TransformBroadcaster brTilt;
        tf::Transform transformTilt;
        transformTilt.setOrigin( tf::Vector3(x, y, -cameraHeight) );
        tf::Quaternion qTilt;
        qTilt.setRPY(PI/2 - curTiltAngleRad, 0, theta);
        transformTilt.setRotation(qTilt);
        brTilt.sendTransform(tf::StampedTransform(transformTilt, ros::Time::now(), "map", "openni_camera_link"));	
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
};
