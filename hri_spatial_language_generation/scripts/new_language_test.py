#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import re
import numpy as np

class
  
def load_rdt_relation_prob_dic(dicfilname):
    with open(dicfilname, 'r') as myfile:
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
  
def load_rdt_content_dic(dicfilename):
    with open(dicfilename, 'r') as myfile:
        data = myfile.readlines()
    dic = {}
    for line in data:
        keyposition = re.search(r'^(.*):', line).span()
        keyword = line[0:keyposition[1]-1];
        text = line[keyposition[1]+1:len(line)-1]
        #print keyword
        #print text
        dic[keyword] = text
    return dic
        
def compute_grounding_relations(grounding, rdtrelationprobdic, relationtypes):
    keywordset = [];
    for i in range(0, len(grounding)-1):
        for j in range(0, len(grounding)-1):
            if i != j:
                keywordset.append(grounding[i] + '-' + grounding[j])
    #print keywordset
    relations = {}
    for k in keywordset:
        if rdtrelationprobdic.get(k):
            probs = rdtrelationprobdic[k]
            maxprobindex = probs.index(max(probs))
            #print probs
            #print probs.index(max(probs))
            relations[k] = relationtypes[maxprobindex]
    return relations
        
def edit_grounding_text(sentence, rdtcontentdic):
    text = ''
    for i in range(len(sentence)):
       clause = rdtcontentdic[sentence[i]]
       print clause
       text = text + ' ' + clause
    print text
        
if __name__ == '__main__':
    #rdtkeyrelationprobfilname = '/home/hri/hri_DATA/spatial_language_generation/rdtkeyrelationprob.txt'
    #rdtcontentfilename = '/home/hri/hri_DATA/spatial_language_generation/rdtcontent.txt'
  
    #grounding = ['bedroom', 'mug', 'room_right_non', 'chair_beside_non', 'non_non_table']
    #relationtypes = ['parent_left', 'parent_right', 'child_left', 'child_right', 'sibling_left', 'sibling_right']
    
    #rdtrelationprobdic = load_rdt_relation_prob_dic(rdtkeyrelationprobfilname)
    #rdtcontentdic = load_rdt_content_dic(rdtcontentfilename)
    #relations = compute_grounding_relations(grounding, rdtrelationprobdic, relationtypes)
    
    
     
    print 'program finished.'
    
