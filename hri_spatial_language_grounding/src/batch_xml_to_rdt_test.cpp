#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <pthread.h>

#include "hri_spatial_language_grounding/CommandProc/Header.h"
#include "hri_spatial_language_grounding/SpatialLanguageGrounding.h"
#include "hri_spatial_language_grounding/PartOfSpeech.h"


using namespace std;

void ShowRobotCmdInfo(ParseToGround _parser)
{
	printf("m_targetObject: %s\n", _parser.m_targetObject.c_str());
	printf("m_targetORoom: %s\n", _parser.m_targetRoom.c_str());
	for (int i = 0; i < _parser.m_RDTNodeSet.size(); i++)
	{
		printf("Node %d:\n", i);
		printf("-m_tar: %s\n", _parser.m_RDTNodeSet[i].m_tar.c_str());
		for (int j = 0; j < _parser.m_RDTNodeSet[i].m_refList.size(); j++)
		{
			printf("-m_refList[%d]: %s\n", j, _parser.m_RDTNodeSet[i].m_refList[j].c_str());
			printf("-m_dirList[%d]: %s\n", j, _parser.m_RDTNodeSet[i].m_dirList[j].c_str());
		}
	}
}

vector<string> GenerateStringVectorRes(ParseToGround _parser)
{
	vector<string> res;
	
	res.push_back("target_object:" + _parser.m_targetObject);
	res.push_back("target_room:" + _parser.m_targetRoom);
	
	for (int i = 0; i < _parser.m_RDTNodeSet.size(); i++)
	{
		res.push_back("tar:" + _parser.m_RDTNodeSet[i].m_tar);
		for (int j = 0; j < _parser.m_RDTNodeSet[i].m_refList.size(); j++)
		{
			res.push_back("ref:" + _parser.m_RDTNodeSet[i].m_refList[j]);
			res.push_back("dir:" + _parser.m_RDTNodeSet[i].m_dirList[j]);
		}
	}
	
 	return res;
}

vector<string> Grounding(string posXmlAddr)
{		
	ParseToGround _parser;
	printf("posXmlAddr:%s\n", posXmlAddr.c_str());
	_parser.ParseXml(posXmlAddr, 1);
	ShowRobotCmdInfo(_parser);
	
	vector<string> grounding = GenerateStringVectorRes(_parser);
	return grounding;
}

int SaveGroundings(string filename, vector<string> grounding)
{
	ofstream myfile;
	myfile.open (filename.c_str());
	for (int i = 0; i < grounding.size(); i++)
		myfile << grounding[i].c_str() << ",";
	myfile.close();
}

int Batch_Grounding_Editing(string srcfolder, string dstfolder)
{
	DIR *dp;
	struct dirent *dirp;
	if((dp  = opendir(srcfolder.c_str())) == NULL) {
	    cout << "Error(" << errno << ") opening " << srcfolder << endl;
	    return errno;
	}
	vector<string> files;
	while ((dirp = readdir(dp)) != NULL) {
		string filedir = string(dirp->d_name);
		if (filedir.find("~") == string::npos && filedir.size() > 3)
		{
			 files.push_back(filedir);
		}
	}
	
	for (int i = 0; i < files.size(); i++) 
	{
		vector<string> grounding = Grounding(srcfolder + files[i]);
		string dstname = files[i].substr(0,files[i].size()-4) + ".txt";
		SaveGroundings(dstfolder + dstname, grounding);
	}
	
	closedir(dp);
	return files.size();
}

int main(int argc, char **argv)
{
	Batch_Grounding_Editing("/home/hri/hri_DATA/new_three_worlds_pos_res/", "/home/hri/hri_DATA/new_three_worlds_grounding_res/");
	return 0;
}






