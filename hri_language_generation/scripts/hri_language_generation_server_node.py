#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import re
import numpy as np
from hri_language_generation.srv import *

#globals
global_rdtkeyrelationprobfilname = '/home/hri/hri_DATA/spatial_language_generation/rdtkeyrelationprob.txt'
global_rdtcontentfilename = '/home/hri/hri_DATA/spatial_language_generation/rdtcontent.txt'
global_relationtypes = ['parent_left', 'parent_right', 'child_left', 'child_right', 'sibling_left', 'sibling_right']

class Node:
  
    def __init__(self, rdt):
        self.parent = None
        self.sibling_left = None
        self.sibling_right = None
        self.child_left = None
        self.child_right = None
        self.rdt = rdt
        self.ps = False

    def insert_node(self, nd, pose):
        if pose == "parent_left":
	    self.child_right = nd
	    nd.parent = self
	if pose == "parent_right":
	    self.child_left = nd
	    nd.parent = self
	if pose == "sibling_right":
	    self.sibling_left = nd
	    nd.sibling_right = self
	if pose == "sibling_left":
	    self.sibling_right = nd
	    nd.sibling_left = self
  
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
        
def compute_grounding_relations(groundings, rdtrelationprobdic, relationtypes):
    keywordset = [];
    for i in range(0, len(groundings)-1):
        for j in range(0, len(groundings)-1):
            if i != j:
                keywordset.append(groundings[i] + '-' + groundings[j])
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
    return text
    
def build_grounding_graph(groundings, relations):
    print groundings
  
    nodelist = [None] * len(groundings)
    for i in range(len(groundings)):
        nodelist[i] = Node(groundings[i]);

    for i in range(len(nodelist)):
        for j in range(len(nodelist)):
	    if i != j:
	        keyword = nodelist[i].rdt + '-' + nodelist[j].rdt
	        if keyword in relations.keys():
	            nodelist[i].insert_node(nodelist[j], relations[keyword])
    
    return nodelist
  
def inorder_traversal_graph(nodelist):
    sentence = []
    print nodelist
    #for n in nodelist:
        #nd = n
        #print nd
        #print nd.rdt
        #if nd.parent != None:
	    #print nd.parent.rdt
	#if nd.sibling_left != None:
            #print nd.sibling_left.rdt
        #if nd.sibling_right != None:
            #print nd.sibling_right.rdt
        #if nd.child_left != None:
            #print nd.child_left.rdt
        #if nd.child_right != None:
            #print nd.child_right.rdt
        #print nd.ps
        ####if n.parent == None and n.sibling_left == None:
	    ####nd = n
	    ####break
	  
    #return        
    
    for n in nodelist:
        print n.rdt
        nd = n
	if nd.sibling_left != None:
	    nd = nd.sibling_left
	    if nd.ps == False:
		nd.ps = True
		sentence.append(nd.rdt)
		print sentence
	else:
	    if nd.child_left != None:
		nd = nd.child_left
		if nd.ps == False:
		    nd.ps = True
		    sentence.append(nd.rdt)
		    print sentence
	    else:
		if nd.ps == False:
		    nd.ps = True
		    sentence.append(nd.rdt)
		    print sentence
		if nd.child_right != None:
		    nd = ne.sibling_right
		    if nd.ps == False:
			nd.ps = True
			sentence.append(nd.rdt)
			print sentence
		else: 
		    if nd.sibling_right != None:
			if nd.ps == False:
			    nd.ps = True
			    sentence.append(nd.rdt)
			    print sentence
		    if nd.parent != None:
			nd = nd.parent
			nd.ps = True
			sentence.append(nd.rdt)
			print sentence
		        
	if n.ps == False:
	    n.ps = True
	    sentence.append(n.rdt)
	    print sentence
			
    return sentence

def language_generation_example():
    rdtkeyrelationprobfilname = '/home/hri/hri_DATA/spatial_language_generation/rdtkeyrelationprob.txt'
    rdtcontentfilename = '/home/hri/hri_DATA/spatial_language_generation/rdtcontent.txt'
  
    groundings = ['bedroom', 'mug']
    relationtypes = ['parent_left', 'parent_right', 'child_left', 'child_right', 'sibling_left', 'sibling_right']
    
    rdtrelationprobdic = load_rdt_relation_prob_dic(rdtkeyrelationprobfilname)
    rdtcontentdic = load_rdt_content_dic(rdtcontentfilename)
    relations = compute_grounding_relations(groundings, rdtrelationprobdic, relationtypes)
    
    print relations
    
    graph = build_grounding_graph(groundings, relations)
    sentence = inorder_traversal_graph(graph)
    
    edit_grounding_text(sentence, rdtcontentdic) 
    
    print 'spatial description generated!'
    
def handle_language_generation(req):
    groundings = req.groundings;
    rdtrelationprobdic = load_rdt_relation_prob_dic(global_rdtkeyrelationprobfilname)
    rdtcontentdic = load_rdt_content_dic(global_rdtcontentfilename)
    relations = compute_grounding_relations(groundings, rdtrelationprobdic, global_relationtypes)
    
    print relations
    
    graph = build_grounding_graph(groundings, relations)
    sentence = inorder_traversal_graph(graph)

    language = edit_grounding_text(sentence, rdtcontentdic) 
    print 'spatial description generated!:%s'%language  
    
    return GenerateSpatialLanguageResponse(language)
    
def language_generation_server():
    rospy.init_node('hri_language_generation_server')
    s = rospy.Service('hri_language_generation', GenerateSpatialLanguage, handle_language_generation)
    print "Ready to generate spatial language."
    rospy.spin()

if __name__ == '__main__':
    #language_generation_example()
    language_generation_server()
    
