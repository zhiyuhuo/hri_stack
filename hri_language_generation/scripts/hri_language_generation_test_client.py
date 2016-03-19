#!/usr/bin/env python

import sys
import rospy
from hri_language_generation.srv import *

def call_for_language_generation_client(groundings):
    rospy.wait_for_service('hri_language_generation')
    try:
        spatial_language = rospy.ServiceProxy('hri_language_generation', GenerateSpatialLanguage)
        resp1 = spatial_language(groundings)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    groundings = ['bedroom', 'mug', 'bed_right_non', 'non_non_table']
    print call_for_language_generation_client(groundings)
