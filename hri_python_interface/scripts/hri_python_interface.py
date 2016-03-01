#!/usr/bin/env python
import roslib;roslib.load_manifest('hri_python_interface')
import rospy
import sys
from std_msgs.msg import String
from twitter import *
	

if __name__ == "__main__":
    rospy.init_node('hri_python_interface_client')
    print "start hri_python_interface_client node"

    t = Twitter(
    auth=OAuth(token, token_key, con_secret, con_secret_key))

    pub = rospy.Publisher('/recognizer/output', String, queue_size=10)
    msg = "the message"
    pub.publish(msg)
    rospy.spin();
      
