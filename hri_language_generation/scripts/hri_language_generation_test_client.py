#!/usr/bin/env python

import sys
import rospy
from hri_language_generation.srv import *

def call_for_language_generation_client(groundings):
    rospy.wait_for_service('hri_language_generation')
    try:
        spatial_language = rospy.ServiceProxy('hri_language_generation', GenerateSpatialLanguage)
        resp1 = spatial_language(groundings)
        return resp1.language
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    
def read_groundings_file(fname):
    with open(fname) as f:
        mylist = f.read().splitlines() 
    print mylist
    return mylist

if __name__ == "__main__":
#    groundings = ['livingroom', 'mug', 'room_center_non', 'couch_front_non', 'non_non_table']
    groundings = read_groundings_file('/home/hri/hri_DATA/test/groundings_for_language_generation/0.txt')
    print call_for_language_generation_client(groundings)
