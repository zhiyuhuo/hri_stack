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


if __name__ == "__main__":
    textfile = "/home/hri/hri_DATA/four_worlds_spatial_descriptions.txt"
    f = open(textfile, 'r')
    txt = f.read()
    lines = txt.split('\n')
    lines.remove("")
    print lines
    typeindices = []
    for i in range(len(lines)):
        if ':' in lines[i]:
	    typeindices = typeindices + [i]
    typeindices + [len(lines)]
    print typeindices
    for i in range(len(typeindices)-1):
        worldobjstr = lines[typeindices[i]]
        worldobjstr = worldobjstr[:-1]
        print worldobjstr
        for j in range(typeindices[i]+1, typeindices[i+1]):
	    linestr = lines[j]
	    linestr.replace(",", " ")
	    linestr.replace(".", " ")
	    txtfile = "/home/hri/hri_DATA/four_worlds_pos_res/" + worldobjstr + "_" + str(j - (typeindices[i]+1)) + "_rawcmd.txt"
	    with open(txtfile, "w") as text_file:
	        text_file.write(linestr)
	    xmlfile = "/home/hri/hri_DATA/four_worlds_pos_res/" + worldobjstr + "_" + str(j - (typeindices[i]+1)) + ".xml"
	    print [xmlfile, linestr]
	    tree = raw_to_xml(linestr,xmlfile)
	    
    #tree = []
    #xmlfile = "/home/hri/hri_DATA/template.xml"
    #tree = raw_to_xml(req.str, xmlfile)
    #print xmlfile
    #print "end chunking ~~~~~~~~~~~"
    #return xmlfile
      
