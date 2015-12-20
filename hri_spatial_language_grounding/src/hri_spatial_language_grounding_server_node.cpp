#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <pthread.h>

#include "std_msgs/String.h"
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

vector<string> generate_resp(ParseToGround _parser)
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

bool Grounding(hri_spatial_language_grounding::SpatialLanguageGrounding::Request  &req, hri_spatial_language_grounding::SpatialLanguageGrounding::Response &res)
{
	ParseToGround _parser;
	string spatialCommandStr = req.str;
	string chunkingTreeFileNameStr = "";

	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<hri_spatial_language_grounding::PartOfSpeech>("hri_part_of_speech");
	hri_spatial_language_grounding::PartOfSpeech srv;
	srv.request.str = spatialCommandStr;
	
	if (client.call(srv))
	{
		cout << "msg receive: " << (string)srv.response.str << endl;;
		chunkingTreeFileNameStr = (string)srv.response.str;
	}
	else
	{
		ROS_ERROR("Failed to call service SpatialLanguageGrounding");
		return 1;
	}
		
	string posXmlAddr = chunkingTreeFileNameStr;
	printf("posXmlAddr:%s\n", posXmlAddr.c_str());
	_parser.ParseXml(posXmlAddr, 1);
	ShowRobotCmdInfo(_parser);
	
	vector<string> groundings = generate_resp(_parser);
	res.strarr = groundings;
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "hri_spatial_language_grounding_server_node");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("hri_spatial_language_grounding", Grounding);
	ROS_INFO("Ready to ground spatial language.");
	ros::spin();

	return 0;
}





