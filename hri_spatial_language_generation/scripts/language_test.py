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
        print keyword
        #print contents[0]
        dic[keyword] = contents[0]
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
            print probs
            print probs.index(max(probs))
            relations[k] = relationtypes[maxprobindex]
    return relations
        
def arrange_groundings(grounding, relations):
    sentence = grounding
    for k in relations.keys():
        if 'parent' in k:
            ws = re.split(r'-', k)
            print ws
            print sentence
            w1 = ws[0]
            w2 = ws[1]
            arrange_one_grounding(w1, w2, sentence, relations[k])
    for k in relations.keys():
        if 'sibling' in k:
            ws = re.split(r'-', k)
            print ws
            print sentence
            w1 = ws[0]
            w2 = ws[1]
            arrange_one_grounding(w1, w2, sentence, relations[k])
    for k in relations.keys():
        if 'child' in k:
            ws = re.split(r'-', k)
            print ws
            print sentence
            w1 = ws[0]
            w2 = ws[1]
            arrange_one_grounding(w1, w2, sentence, relations[k])
    print sentence
    
def arrange_one_grounding(w1, w2, s, r):
    if r == 'parent_left':
        s.remove(w2)
        i1 = s.index(w1)
        s.insert(i1-1,w2)
    if r == 'parent_right':
        s.remove(w2)
        i1 = s.index(w1)
        s.insert(i1-1,w2)
    if r == 'child_left':
        s.remove(w2)
        i1 = s.index(w1)
        s.insert(i1-1,w2)
    if r == 'child_right':
        s.remove(w2)
        i1 = s.index(w1)
        s.insert(i1-1,w2)
    if r == 'sibling_left':
        s.remove(w2)
        i1 = s.index(w1)
        s.insert(i1-1,w2)
    if r == 'sibling_left':
        s.remove(w2)
        i1 = s.index(w1)
        s.insert(i1-1,w2)
    
        
if __name__ == '__main__':
    grounding = ['bedroom', 'mug', 'room_right_non', 'chair_beside_non', 'non_non_table']
    relationtypes = ['parent_left', 'parent_right', 'child_left', 'child_right', 'sibling_left', 'sibling_right']
    rdtrelationprobdic = load_rdt_relation_prob_dic()
    rdtcontentlist = load_rdt_content_dic()
    relations = compute_grounding_relations(grounding, rdtrelationprobdic, relationtypes)
    arrange_groundings(grounding, relations)
    print 'program finished.'
    
