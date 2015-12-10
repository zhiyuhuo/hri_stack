#!/usr/bin/env python
import roslib;roslib.load_manifest('hri_spatial_language_grounding')
import re
import nltk
import nltk.chunk
import numpy
import random
import nltk.tag
import sys
#from nltk.tag import brill_modified
from nltk.tag import brill
#from nltk.etree.ElementTree import ElementTree
import xml.etree
from nltk import Tree
import rospy
from train_parser import *
from std_msgs.msg import String
from hri_spatial_language_grounding.srv import *

tree = []
ifreceive = 0;
xmlfile = "/home/hri/hri_DATA/template.xml";

def handle_part_of_speech(req):
    global ifreceive;
    global xmlfile;
    print req.str
    xmlfile = "/home/hri/hri_DATA/template.xml"
    tree = raw_to_xml(req.str, xmlfile)
    print xmlfile
    print "end chunking ~~~~~~~~~~~"
    return xmlfile
	
def handle_pos_server():
    rospy.init_node('hri_part_of_speech_node')
    s = rospy.Service('hri_part_of_speech', PartOfSpeech, handle_part_of_speech)
    print "ready to pos"
    rospy.spin()

if __name__ == "__main__":
    handle_pos_server()
      
