#!/usr/bin/env python
#import roslib;roslib.load_manifest('hri_python_interface_client')
#import re
#import nltk
#import nltk.chunk
#import numpy
#import random
#import nltk.tag
#import sys
##from nltk.tag import brill_modified
#from nltk.tag import brill
##from nltk.etree.ElementTree import ElementTree
#import xml.etree
#from nltk import Tree
#import rospy
#from train_parser import *
#from std_msgs.msg import String

#tree = []
#ifreceive = 0;
#xmlfile = "/home/hri/hri_DATA/template.xml";

#def callback(data):
    #global ifreceive;
    #global xmlfile;
    #if ifreceive == 0:
	#print data.data
	#xmlfile = "/home/hri/hri_DATA/template.xml"
	#tree = raw_to_xml(data.data, xmlfile)
	#print xmlfile
	#print "end chunking ~~~~~~~~~~~"
	#ifreceive = 1;
    #else:
	#pub = rospy.Publisher('/address', String, queue_size=10)
	#print "xml file ready ~~~~~~~~~~~~~~~~~~"
	#pub.publish(xmlfile)
	

#if __name__ == "__main__":
    #rospy.init_node('part_of_speech')
    #print "start part_of_speech node"
    #rospy.Subscriber("/recognizer/output", String, callback)
    #ifreceive = 0;
    #rospy.spin();
      
