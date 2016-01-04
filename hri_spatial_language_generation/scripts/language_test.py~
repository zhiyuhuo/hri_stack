#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import re
import numpy as np

def load_rdt_relation_prob_matrix():
    matfilname = '/home/hri/hri_DATA/spatial_language_generation/rdtkeyrelationprob.txt'
    with open(matfilname, 'r') as myfile:
        data = myfile.readlines()
    return data
  
def generate_dic_from_rdt_prob_mat(mat):
    dic = {}
    for m in mat:
        line = m
        keyposition = re.search(r'^(.*):', line).span()
        datastr = line[keyposition[1]+1:]
        values = re.split(r'\s+', datastr)
        key = line[0:keyposition[1]-1];
        Lprob = len(values) - 1
        values = values[0: Lprob]
	probs = [float(f) for f in values]
	#print key
	#print probs
	dic[key] = probs
    return dic

        
        
if __name__ == '__main__':
    grounding = ['bedroom', 'mug', 'room_right_non', 'chair_beside_table']
    data = load_rdt_relation_prob_matrix()
    dic = generate_dic_from_rdt_prob_mat(data)
    print 'program finished.'
    