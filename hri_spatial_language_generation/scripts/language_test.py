#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import re
import numpy as np
  
def load_rdt_relation_prob_dic():
    matfilname = '/home/hri/hri_DATA/spatial_language_generation/rdtkeyrelationprob.txt'
    with open(matfilname, 'r') as myfile:
        data = myfile.readlines()
    dic = {}
    for line in data:
        keyposition = re.search(r'^(.*):', line).span()
        datastr = line[keyposition[1]+1:]
        values = re.split(r'\s+', datastr)
        keyword = line[0:keyposition[1]-1];
        Lprob = len(values) - 1
        values = values[0: Lprob]
	probs = [float(f) for f in values]
	#print keyword
	#print probs
	dic[keyword] = probs
    return dic
  
def load_rdt_content_dic():
    dicfilename = '/home/hri/hri_DATA/spatial_language_generation/rdtcontent.txt'
    with open(dicfilename, 'r') as myfile:
        data = myfile.readlines()
    dic = {}
    for line in data:
        keyposition = re.search(r'^(.*):', line).span()
        datastr = line[keyposition[1]+1:]
        contents = re.split(r',+', datastr)
        keyword = line[0:keyposition[1]-1];
        Lprob = len(contents) - 1
        contents = contents[0: Lprob]
        #print keyword
        #print contents[0]
        dic[keyword] = contents[0]
    return dic
        
def compute_structure_energy(grounding, rdtrelationprobdic):
    keywordset = [];
    for i in range(0, len(grounding)-1):
        for j in range(0, len(grounding)-1):
            if i != j:
                keywordset.append(grounding[i] + '-' + grounding[j])
    #print keywordset
    for k in keywordset:
        probs = rdtrelationprobdic[k]
        print probs
        print probs.index(max(probs))
    
        
if __name__ == '__main__':
    grounding = ['bedroom', 'mug', 'room_right_non', 'chair_beside_non' 'non_non_table']
    relationtypes = ['parent_left', 'parent_right', 'child_left', 'child_right', 'sibling_left', 'sibling_right']
    rdtrelationprobdic = load_rdt_relation_prob_dic()
    rdtcontentlist = load_rdt_content_dic()
    compute_structure_energy(grounding, rdtrelationprobdic)
    print 'program finished.'
    
