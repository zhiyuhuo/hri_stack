#!/usr/bin/env python
import roslib;roslib.load_manifest('hri_python_interface')
import rospy
import sys
from std_msgs.msg import String
import fbchat

if __name__ == "__main__":
    rospy.init_node('hri_python_interface_client')
    print "start hri_python_interface_client node"


    
    client = fbchat.Client("100011404486718", "hrihri")
    print "login finished"
    
    msgRec = client.ping()

    pub = rospy.Publisher('/recognizer/output', String, queue_size=10)
    msg = "the message"
    pub.publish(msg)
    rospy.spin();
      
