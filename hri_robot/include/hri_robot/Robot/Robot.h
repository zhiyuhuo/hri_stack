#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include <cstdlib>
#include <vector>

#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "p2os_msgs/MotorState.h"
#include "hri_perception/Env.h"
#include "hri_spatial_language_grounding/SpatialLanguageGrounding.h"
#include "hri_language_generation/GenerateSpatialLanguage.h"
#include "hri_perception/Perception.h"
#include "nav_msgs/SetMap.h"
#include "nav_msgs/GetPlan.h"
#include <vector>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../Geometry/Header.h"
#include "../Universal/Header.h"
#include "Header.h"

#define OCCUPANCYMAP_W 100
#define OCCUPANCYMAP_H 100
#define OCCUPANCYMAP_X -12.5
#define OCCUPANCYMAP_Y -12.5
#define OCCUPANCYMAP_RL 0.25

using namespace std;
using namespace cv;

class RDTNode
{
public:
	RDTNode(){}
	~RDTNode(){}

	string m_tar;
	vector<string> m_refList;
	vector<string> m_dirList;
};


class Ent{
public:
	Ent(){dir = 0; x = 0; y = 0; name = "";}
	~Ent(){}
    
	vector<float> vec;
	float dir;
	string name;
	float x;
	float y;
	float confidence;
    
	int id;
	string tag;
};

class SpR{
public:
	string nameA;
	string nameB;
	
	vector<float> dirhg;
	vector<float> disthg;
	
	vector<float> outdirw;
	vector<float> indirw;
	vector<float> distw;
	
};

class Dct{
public:  
	string nameA;
	string nameB;
	
	vector<float> outdirw;
	vector<float> indirw;
	vector<float> distw;

};

class Robot
{
public:
	Robot();
	~Robot();
	
public: //ros
	ros::NodeHandle* m_nh;
	string m_robot_namespace_;
	
public: // robot state variables
	//linear and angular speed control parameter
	float m_linearSpeed;
	float m_angularSpeed;
	
	float m_angularConstPlatform;
	float m_speedScalePlatform;
	//Robot Position and Orientation
	VecPosition m_posRobot;
	float m_theta;
	VecPosition m_moveTarget;
	float m_turnTarget;
	ros::Publisher m_speedPub;
	int SetRobotVelocity();
	
public: // robot navigation
	int m_path;
	vector<VecPosition> m_pathPoints;
	ros::Subscriber m_poseSub;
	void poseCallback(const nav_msgs::OdometryConstPtr& msg);
	ros::ServiceClient m_setMapClient; 
	ros::ServiceClient m_getPlanClient;
	vector<uint8_t> m_occupancyMap;
	int SetOccupancyMap(int width, int height, double resolution, double originX, double originY);
	vector<VecPosition> CallForPathPlan(VecPosition posStart, VecPosition posTarget);
	
public:	//robot basic action
	int ToPos(VecPosition posTarget);
	int ToAngle(float angleTarget);
	int ToPosAngle(VecPosition posTarget, float angleTarget);
	int ToPositionAvoidObstacles(VecPosition posTarget);
	
public:	//robot sensor data processing
	ros::ServiceClient m_perceptionClient; 
  	ros::Subscriber m_envSub;
	vector<Ent> m_SEList;	
	vector<Ent> m_tempSEList;
	vector<float> LocalToGlobal(float lx, float ly, float lth); //x, y, theta
	void CallForPercepstionService();
	void EnvCallback(const hri_perception::Env::ConstPtr& msg);
	int Perception();
	int UpdateSEMap(vector<Ent> tempEntList);
	int CompareTwoGOs(Ent go1, Ent go2);
	Ent GenerateGlobalEnt(Ent le);
	int DrawOccupancyGrid();
	
public:	//parsing chunking tree
	ros::Subscriber m_cmdSub;
	void cmdstrCallback(const std_msgs::StringConstPtr& msg);
	string m_spatialCommand;
	int m_counter;
	ros::ServiceClient m_groundingClient; 
	int AskGroundingService(string cmd);
	string m_worldName;
	string m_targetRoom;
	string m_targetObject;
	vector<RDTNode> m_RDTList;
	void ShowRobotCmdInfo();
	
public:	//new Behaviors for imitation learning
	vector<float> m_originalRobotPose;
	Mat m_imgOccupancy;
	Mat m_imgGrid;
	vector<vector<string> > m_groundings;
	int m_step;
	string m_cmdID;
	
	vector<float> PixelToCoordinate(int u, int v);
	vector<int> CoordinateToPixel(float x, float y);
	
	
public:	//robot behavior
	bool m_ifGetPerception;
	bool m_ifGetPose;
	
	string m_move;
	string m_action;
	string m_state;
	string m_mission;
	
public:	//Robot Behavior
	float m_currentScore;
	vector<Dct> m_currentDct;
	int RunCommand(vector<string> cmd);
	int Search90();
	int SearchAround();
	int GotoRoom(string worldAndroom);
	int RDTIndoorNavigation(RDTNode rdt);
	vector<Dct> ReadDetector(vector<string> cmd);
	float ScoreCurrentState(vector<Dct> decisionSpatialRelations);
	float IterateltSearchTarget(vector<Dct> decisionSpatialRelations);
	float IterateSearchTargetOptimized(vector<Dct> decisionSpatialRelations);
	vector<vector<Ent> > ListEntitiesSetCombination(vector<string> requiredEntsNames, vector<Ent> myEnts);
	
public:	//World State Feature
	vector<Ent> m_entities;
	Ent GetRobotEntity(float x, float y, float theta);
	vector<Ent> m_LEList;
	int ImportLEList(string rootDir);
	Ent GetWallEntity(float x, float y, float theta);
	Ent GetLE(string fname, string lename);
	SpR GetSpatialRelationB2A(Ent A, Ent B);	
	vector<float> GetDistanceBetweenTwoSpatialFeature(vector<float> dirhg1, vector<float> disthg1, vector<float> dirhg2, vector<float> disthg2);
	float GetResponseOfADetector(SpR sp, Dct dct);
	vector<vector<float> > DirectionWeights(vector<float> hof);
	vector<float> DistanceWeights(vector<float> hd);
	vector<Ent> GetVisiableEntities();
	
public:	//Robot Strategy 
	VecPosition m_posRobotLast;
	float m_pathLength;
	int ConnectToServer();
	int RunNode();
	
	int TestFromRDT(vector<string> groundings);
	int HomeFetchTask();
	int DecisionMaking();
	int Test();
	int TestPathGeneration();
	int TestRead();
	int UpdatePerception();
	int KeyboardControlForLanguageGeneration();
	int AutomaticLanguageGenerationFromVideo(string targetObject);
	
	int BuildGroundingList(vector<string> groundings);
	int BuildFakeGroundingList();
	int FormatGroundings();
	
public:	//Grounding Generation (Generate RDT from Path)
  	ros::ServiceClient m_generatingLanguageClient; 
	map<string, vector<Dct> > LoadGroundingTypesList(string fileName);
	vector<string> GenerateStaticDescription(map<string, vector<Dct> > dctMap);
	vector<string> GenerateDynamicDescription(float addresseeDir, vector<VecPosition> pathPoints, map<string, vector<Dct> > dctMap);
	float ScoreStateToOneGrounding(vector<float> CRPose, vector<float> ORPose, vector<Dct> decisionSpatialRelations, bool isStatic);
	vector<string> AdjustGroundingsFormatToLGServer(vector<string> groundings);
	void AnalyseEntityRelation();
	void LoadGroundingC0Appear();
};

Robot::Robot()
{
	m_counter = 0;
	m_step = 0;
	m_cmdID = "";
	
	m_move = "init";
	m_action = "init";
	m_state = "init";	//temporally out of use
	m_mission = "init";
	
	m_originalRobotPose.push_back(0);
	m_originalRobotPose.push_back(0);
	m_originalRobotPose.push_back(0);
	
	m_linearSpeed = 0;
	m_angularSpeed = 0;
	
	m_angularConstPlatform = 1.0;
	m_speedScalePlatform = 1.0;
	
	m_posRobot.SetX(0);
	m_posRobot.SetY(0);
	m_theta = PI / 2;
	m_posRobotLast = m_posRobot;
	
	m_imgOccupancy = Mat::zeros(200, 200, CV_8UC1);
	m_imgGrid = Mat::zeros(200, 200, CV_8UC3);
	m_pathLength = 0;
	m_spatialCommand = "";
	
	m_ifGetPerception = false;
	m_ifGetPose = false;
	
}

Robot::~Robot()
{

}

int Robot::ConnectToServer()
{
	if (!ros::isInitialized())
	{
		int argc = 0;
		char** argv = NULL;
		ros::init(argc,argv,"hri_robot_node",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
	}
	
	this->m_robot_namespace_ = "";
	m_nh = new ros::NodeHandle(this->m_robot_namespace_);
	
	//sub and pub
	string platformStr;
	string poseTopicStr;
	string cmdvelTopicStr;
	
	string key;
	if (m_nh->searchParam("robot_platform", key))
	{
	    m_nh->getParam(key, platformStr);
	}
	if (platformStr.compare("physical") == 0)
	{
	    poseTopicStr = "/pose";
	    cmdvelTopicStr = "/cmd_vel";
	}
	else
	{
	    poseTopicStr = "/hri_robot/odom";
	    cmdvelTopicStr = "/hri_robot/cmd_vel";  
	    m_angularConstPlatform = -1.0;
	    m_speedScalePlatform = 2.0;
	}
	
	m_poseSub = m_nh->subscribe(poseTopicStr.c_str(), 1000, &Robot::poseCallback, this);
	m_envSub = m_nh->subscribe("/env", 10, &Robot::EnvCallback, this);
	m_speedPub = m_nh->advertise<geometry_msgs::Twist>(cmdvelTopicStr.c_str(), 100);
	m_cmdSub = m_nh->subscribe("recognizer/output", 10, &Robot::cmdstrCallback, this);
	
	
	cout <<poseTopicStr << " " << cmdvelTopicStr << endl;
	
	//clients
	m_groundingClient = m_nh->serviceClient<hri_spatial_language_grounding::SpatialLanguageGrounding>("hri_spatial_language_grounding");
	m_generatingLanguageClient = m_nh->serviceClient<hri_language_generation::GenerateSpatialLanguage>("hri_language_generation");
	m_perceptionClient = m_nh->serviceClient<hri_perception::Perception>("hri_perception");
	m_setMapClient = m_nh->serviceClient<nav_msgs::SetMap>("nav_set_map");
	m_getPlanClient = m_nh->serviceClient<nav_msgs::GetPlan>("nav_get_plan");
	
	//LE
	string worldNameStr;
	if (m_nh->searchParam("world_name", key))
	{
	    m_nh->getParam(key, worldNameStr);
	}
	
	//Get Le
	m_worldName = worldNameStr;
	string LEDir("/home/hri/hri_DATA/LE_" + worldNameStr);
	cout << "loading le from " << LEDir << endl;
	ImportLEList(LEDir.c_str());
	
	return 1;
}

vector<float> Robot::LocalToGlobal(float lx, float ly, float lth)
{
	vector<float> res(3, 0);
	float posX = m_posRobot.GetX();
	float posY = m_posRobot.GetY();
	float th = m_theta - PI/2;
	float gx = cos(th) * lx + -sin(th) * ly + posX;
	float gy = sin(th) * lx + cos(th) * ly + posY;
	res[0] = gx;
	res[1] = gy;
	res[2] = lth + th;
	while (res[2] > 2 * PI)
	{
		res[2] -= 2 * PI;
	}
	while (res[2] < 0)
	{
		res[2] += 2 * PI;
	}
	
	return res;
}

vector<float> Robot::PixelToCoordinate(int u, int v)
{
	vector<float> res(2, 0);
	res[0] = (u - 100.0) * 0.1;
	res[1] = -(v - 100.0) * 0.1;
	return res;
}

vector<int> Robot::CoordinateToPixel(float x, float y)
{
	vector<int> res(2, 0);
	res[0] = (int)(x / 0.1 + 100);
	res[1] = (int)(100 - y / 0.1);
	return res;	
}

#endif
