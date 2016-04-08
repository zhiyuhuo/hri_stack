#!/usr/bin/env python  
import math
import roslib
roslib.load_manifest('hri_tf')
import rospy

import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

PI = 3.1416
x = 0
y = 0
z = 0
theta = 0
camera_height = 1.05
cur_tilt = 0

def handle_pose(msg):
    global x
    global y
    global theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    rotz = math.atan2(2*qw*qz, 1-2*qz*qz)
    theta = rotz;
	    
    while (theta < 0):
	    theta = theta + 2*PI

    while (theta > 2*PI):
	    theta = theta - 2*PI;

def handle_tilt(msg):
    cur_tilt = msg.data

if __name__ == '__main__':
    rospy.init_node('hri_tf_kinect')
    rospy.Subscriber('/hri_robot/odom', Odometry, handle_pose)
    rospy.Subscriber('/cur_tilt_angle', Float64, handle_tilt)
                   
    rate = rospy.Rate(10)  
    while not rospy.is_shutdown():
        cur_tilt_rad = cur_tilt * PI / 180
        br = tf.TransformBroadcaster()
        #print [x, y, theta, cur_tilt, camera_height, PI]
        br.sendTransform((x, camera_height, y), tf.transformations.quaternion_from_euler(PI/2 - cur_tilt_rad, theta, 0), rospy.Time.now(), "map", "openni_camera_link")

        rate.sleep();
                    