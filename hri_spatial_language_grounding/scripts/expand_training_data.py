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

def copy_and_replace_word(string):
    ifchange = 0
    if string.find("chairs") > 0:
        string = string.replace("chairs", "chair")
        print string
        ifchange = 1
    if string.find("bedroom") > 0 or string.find("living") > 0:
        roomstrset = ["office", "studio"]
        string = string.replace("bedroom", roomstrset[random.randrange(len(roomstrset))])
        print string
        ifchange = 1
    if string.find("living") > 0 or string.find("living") > 0:
        roomstrset = ["office", "studio", "meeting"]
        string = string.replace("living", roomstrset[random.randrange(len(roomstrset))])
        print string
        ifchange = 1
    return string, ifchange
  
def add_ormtp(string):
    ifchange = 0
    roomstrset = ["\r\n<ORMTP>\r\n<VB>go</VB>\r\n<TO>to</TO>\r\n<DT>the</DT>\r\n<RM>office</RM>\r\n</ORMTP>",
		  "\r\n<ORMTP>\r\n<VB>go</VB>\r\n<TO>to</TO>\r\n<DT>the</DT>\r\n<RM>bedroom</RM>\r\n</ORMTP>",
		  "\r\n<ORMTP>\r\n<VB>go</VB>\r\n<TO>to</TO>\r\n<DT>the</DT>\r\n<RM>studio</RM>\r\n</ORMTP>",
		  "\r\n<ORMTP>\r\n<VB>go</VB>\r\n<TO>to</TO>\r\n<DT>the</DT>\r\n<NN>office</NN>\r\n<RM>room</RM>\r\n</ORMTP>",
	          "\r\n<ORMTP>\r\n<VB>go</VB>\r\n<TO>to</TO>\r\n<DT>the</DT>\r\n<NN>living</NN>\r\n<RM>room</RM>\r\n</ORMTP>",
	          "\r\n<ORMTP>\r\n<VB>go</VB>\r\n<TO>to</TO>\r\n<DT>the</DT>\r\n<NN>studio</NN>\r\n<RM>room</RM>\r\n</ORMTP>",]
    string = string[0:5] + roomstrset[random.randrange(len(roomstrset))] + string[5:-1]
    print string
    return string

if __name__ == "__main__":
    originalfile = "/home/hri/hri_DATA/Indoor149.xml"
    f = open(originalfile, 'r')
    txt = f.read()
    
    grp1 = [i for i in range(len(txt)) if txt.startswith('<S>', i)]
    grp2 = [i for i in range(len(txt)) if txt.startswith('</S>', i)]
    
    print grp1
    print grp2
    lines = []
    
    for i in range(len(grp1)):
        line = [txt[grp1[i]-2:grp2[i]+6]]
        lines = lines + line
    print lines
    
    extended = []
    for l in lines:
      
        #string, ifchange = copy_and_replace_word(l)
        #if ifchange == 1:
            #extended = extended + [string]
            
        string = add_ormtp(l)
        extended = extended + [string]
    print extended
    print len(extended)
    
    lines = lines + extended
    print lines
    print len(lines)
    linesstr = ''.join(lines)
    newtxt = "<ALL>\r\n" + linesstr + "</ALL>"
    print newtxt
    f = open("/home/hri/hri_DATA/ExtendedTemplate2.xml", "w")
    f.write(newtxt)
    f.close()
		

      
