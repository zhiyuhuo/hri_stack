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
#include "../CommandProc/Header.h"
#include "Header.h"

using namespace std;
using namespace cv;

struct Fur
{
	string name;
	vector<float> posedim; //x, y, theta, dimension x, dimension y
};

struct Ent{
	vector<float> vec;
	float dir;
	string name;
};

struct SpR{
	string nameA;
	string nameB;
	
	vector<float> dirhg;
	vector<float> disthg;
	
	vector<float> outdirw;
	vector<float> indirw;
	vector<float> distw;
	
};

struct Dct{
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
	Robot(string rootDir, string saveDir, string mapFile);
	~Robot();
	
public:
	//linear and angular speed control parameter
	float m_linearSpeed;
	float m_angularSpeed;

	//Robot Position and Orientation
	VecPosition m_posRobot;
	float m_theta;

public:	//robot basic action
	int ToPos(VecPosition posTarget);
	int ToAngle(float angleTarget);
	int ToPosAngle(VecPosition posTarget, float angleTarget);
	int ToPositionAvoidObstacles(VecPosition posTarget);
	int SearchWide();
	int SearchSpin();
	
public:	//robot sensor data processing
	int m_laserUnitNumber;
	vector<vector<float> > m_laserData;
	vector<float> LocalToGlobal(float lx, float ly, float lth); //x, y, theta
	int Perception();
	int UpdateOccupyMap();
	
public:	//parsing chunking tree
	VecPosition m_moveTarget;
	float m_turnTarget;
	int m_counter;
	
	vector<RDTNode> m_RDTNodeChain;	
	string m_targetRoom;
	string m_targetObject;
	
public:	//new Behaviors for imitation learning
	string m_humanCmd;
	vector<string> m_cmdWords;
	vector<float> m_originalRobotPose;

	Mat m_imgmap;
	Mat m_imgmask;
	Mat m_imgrobot;
	Mat m_imgshow;
	Mat m_imglattice;
	
	Mat m_imgOccupy;
	
	int m_step;
	string m_cmdID;
	
	vector<float> PixelToCoordinate(int u, int v);
	vector<int> CoordinateToPixel(float x, float y);
	void ExploreMap();
	void ShowRobotPose();
	bool DetectObstacle(int x1, int y1, int x2, int y2);
	
	vector<vector<string> > GetGroundingInfo(string fileName);
	vector<vector<string> > m_groundings;
	vector<vector<string> > m_cmdRec;
	vector<vector<float> > m_poseRec;
	vector<int> m_fr;
	vector<vector<int> > m_frRec;
	string m_key;
	
public:	//robot behavior
	string m_move;
	string m_action;
	string m_state;
	string m_mission;
	
// 	string m_currentTar;
// 	string m_currentRef;
// 	string m_currentDir;
	
	//Human Control Robot
	int KeyBoardAction();
	int KeyBoardControl();
	int PathRec();
	
	//Robot Behavior
	float m_currentScore;
	vector<Dct> m_currentDct;
	int RunCommand(vector<string> cmd);
	int GotoRoom(string room);
	int RDTIndoorNavigation(RDTNode rdt);
	vector<Dct> ReadDetector(vector<string> cmd);
	float ScoreCurrentState(vector<Dct> decisionSpatialRelations);
	float IterateltSearchTarget(vector<Dct> decisionSpatialRelations);
	vector<vector<Ent> > ListEntitiesSetCombination(vector<string> requiredEntsNames, vector<Ent> myEnts);
	
	//World State Feature
	map<int, Fur> m_furLib;
	void ReadGOInformation(string filename);
	vector<Ent> m_entities;
	Ent GetRobotEntity(float x, float y, float theta);
	Ent GetGoEntity(int frid);
	Ent GetWallEntity(string targetRoom);
	Ent GetRoomEntity(string roomName);
	SpR GetSpatialRelationB2A(Ent A, Ent B);	
	vector<float> GetDistanceBetweenTwoSpatialFeature(vector<float> dirhg1, vector<float> disthg1, vector<float> dirhg2, vector<float> disthg2);
	float GetResponseOfADetector(SpR sp, Dct dct);
	vector<vector<float> > DirectionWeights(vector<float> hof);
	vector<float> DistanceWeights(vector<float> hd);
	vector<Ent> GetVisiableEntities();
	
	//Robot Intelligence 
	VecPosition m_posRobotLast;
	string m_saveDir;
	float m_thetaLast;
	float m_pathLength;
	int DecisionMaking();
	int Test();
	int TestRead();
	int UpdatePerception();
	
public://Show Robot State
  	//This part is only for testing
	int BuildFakeTestingData();
	int ShowRobotState();
};

Robot::Robot()
{ 
	m_counter = 0;
	m_step = 0;
	m_cmdID = "";
	
	m_move = "init";
	m_action = "stop";
	m_state = "init";	//temporally out of use
	m_mission = "init";
	
	m_posRobot.SetX(0);
	m_posRobot.SetY(0);
	m_theta = PI / 2;
	m_posRobotLast = m_posRobot;
	m_thetaLast = m_theta;
	
	m_imgmap = imread("/home/hri/hri_DATA/pbd/map_trainer.bmp");
	m_imgmask = imread("/home/hri/hri_DATA/pbd/mask.bmp");
	m_imglattice = imread("/home/hri/hri_DATA/pbd/lattice.bmp");
	m_imgrobot = imread("/home/hri/hri_DATA/pbd/robot.bmp");
	m_imgshow = m_imgmask.clone();
	Mat colorMat = imread("/home/hri/hri_DATA/pbd/room.bmp");
	cvtColor(colorMat, m_imgOccupy, CV_BGR2GRAY);
	
	m_saveDir = "/home/hri/hri_DATA/pbd/Human_Demo_Rec_4/";
	m_pathLength = 0;
	
	m_fr.push_back(-1);
	m_fr.push_back(201);
	m_fr.push_back(202);
}

Robot::Robot(string rootDir, string saveFolder, string mapFile)
{
	m_counter = 0;
	m_step = 0;
	m_cmdID = "";
	
	m_move = "init";
	m_action = "stop";
	m_state = "init";	//temporally out of use
	m_mission = "init";
	
	m_posRobot.SetX(0);
	m_posRobot.SetY(0);
	m_theta = PI / 2;
	m_posRobotLast = m_posRobot;
	m_thetaLast = m_theta;
	
	string imgmapDir = rootDir + "map_trainer.bmp";
	string maskDir = rootDir + "mask.bmp";
	string latticeDir = rootDir + "lattice.bmp";
	string robotDir = rootDir + "robot.bmp";
	string colorDir = rootDir + "room.bmp";
	
	m_imgmap = imread(imgmapDir.c_str());
	m_imgmask = imread(maskDir.c_str());
	m_imglattice = imread(latticeDir.c_str());
	m_imgrobot = imread(robotDir.c_str());
	m_imgshow = m_imgmask.clone();
	Mat colorMat = imread(colorDir.c_str());
	cvtColor(colorMat, m_imgOccupy, CV_BGR2GRAY);
	
	m_saveDir = rootDir + saveFolder + "/";
	m_pathLength = 0;
	
	m_fr.push_back(-1);
	m_fr.push_back(201);
	m_fr.push_back(202);
}

Robot::~Robot()
{
	
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

int Robot::BuildFakeTestingData()
{
	m_targetRoom = "living room";
	m_targetObject = "fork";
	
	RDTNode node1;
	node1.m_tar = "table";
	node1.m_refList.push_back("move");
	node1.m_dirList.push_back("right");
// 	node1.m_refList.push_back("robot");
// 	node1.m_dirList.push_back("right");
	
//  	RDTNode node2;hai
//  	node2.m_tar = "table";
//  	node2.m_refList.push_back("move");
//  	node2.m_dirList.push_back("right");
	
//  	RDTNode node3;
//  	node3.m_tar = "";
//  	node3.m_refList.push_back("wall");
//  	node3.m_dirList.push_back("front");
// 	
//  	RDTNode node4;
//  	node4.m_tar = "";
//  	node4.m_refList.push_back("move");
//  	node4.m_dirList.push_back("left");
	
	m_RDTNodeChain.push_back(node1);
// 	m_RDTNodeChain.push_back(node2);
// 	m_RDTNodeChain.push_back(node3);
// 	m_RDTNodeChain.push_back(node4);
	return 0;
}

vector<float> Robot::PixelToCoordinate(int u, int v)
{
	vector<float> res(2, 0);
	res[0] = (u - 250.0) * 0.04;
	res[1] = -(v - 250.0) * 0.04;
	return res;
}

vector<int> Robot::CoordinateToPixel(float x, float y)
{
	vector<int> res(2, 0);
	res[0] = (int)(x / 0.04 + 250);
	res[1] = (int)(250 - y / 0.04);
	return res;	
}

bool Robot::DetectObstacle(int x1, int y1, int x2, int y2)
{
	int nid[] = { 0, -1, -1, 0, 1, 0, 0, 1 };
	float dmin = sqrt((float)(pow(x2-x1, 2)+ pow(y2-y1, 2)));
	int xp = x1;
	int yp = y1;
	while(dmin > 0)
	{
		for (int i = 0; i < 4; i++)
		{
			int pnix = xp + nid[2*i]; 
			int pniy = yp + nid[2*i+1];
			float dist = (float)(sqrt(pow(x2-pnix, 2) + pow(y2-pniy, 2)));
			//printf("%f %f\n", dist, dmin);
			if ( dist < dmin )
			{
				dmin = dist;
				xp = pnix;
				yp = pniy;
				if (m_imgmap.at<Vec3b>(yp, xp)[0] == 255
				  && m_imgmap.at<Vec3b>(yp, xp)[1] == 255
				  && m_imgmap.at<Vec3b>(yp, xp)[2] == 255
				)
				{
					return false;
				}
			} 
		}
	}
	return true;
}

void Robot::ExploreMap()
{
	int robotPixelX;
	int robotPixelY;
	float mindist = 1000;
	for (int i = 0; i < m_imgmap.cols; i++)
	{
		for (int j = 0; j < m_imgmap.rows; j++)
		{
			vector<float> p = PixelToCoordinate(i, j);
			float dist = sqrt(pow(p[0]-m_posRobot.GetX(), 2) + pow(p[1]-m_posRobot.GetY(), 2));
			if (dist < mindist)
			{
				mindist = dist;
				robotPixelX = i;
				robotPixelY = j;
			}			
		}
	}
	for (int i = 0; i < m_imgmap.cols; i++)
	{
		for (int j = 0; j < m_imgmap.rows; j++)
		{
			vector<float> p = PixelToCoordinate(i, j);
			float dist = sqrt(pow(p[0]-m_posRobot.GetX(), 2) + pow(p[1]-m_posRobot.GetY(), 2));
			float dir = atan2(p[1] - m_posRobot.GetY(), p[0] - m_posRobot.GetX());
			if (dist < 2.5 && dist > 0.5)
			{
				float deltaDir = dir - m_theta;
				if (deltaDir > PI)
				{
					deltaDir = deltaDir - 2 * PI;
				}
				if (deltaDir < -PI)
				{
					deltaDir = deltaDir + 2 * PI;
				}
				if (abs(deltaDir) < PI/8)
				{
					if (DetectObstacle(i, j, robotPixelX, robotPixelY))
					{
						m_imgmask.at<Vec3b>(j, i) = m_imgmap.at<Vec3b>(j, i);
					}
				}
			}
			m_imgshow.at<Vec3b>(j, i) = m_imgmask.at<Vec3b>(j, i);
		}
	}

}

void Robot::ShowRobotPose()
{
	int robotPixelX;
	int robotPixelY;
	float mindist = 1000;
	for (int i = 0; i < m_imgmap.cols; i++)
	{
		for (int j = 0; j < m_imgmap.rows; j++)
		{
			vector<float> p = PixelToCoordinate(i, j);
			float dist = sqrt(pow(p[0]-m_posRobot.GetX(), 2) + pow(p[1]-m_posRobot.GetY(), 2));
			if (dist < mindist)
			{
				mindist = dist;
				robotPixelX = i;
				robotPixelY = j;
			}			
		}
	}  
	for (int i = 0; i < m_imgrobot.cols; i++)
	{
		for (int j = 0; j < m_imgrobot.rows; j++)
		{ 
			Vec3b intensity;
			intensity.val[0] = 0;
			intensity.val[1] = 0;
			intensity.val[2] = 0;
			m_imgrobot.at<Vec3b>(j, i) = intensity;
		}
	}  
  
	circle(m_imgrobot, Point(robotPixelX, robotPixelY), 5, CV_RGB(255, 125, 125), 2, 8);
	float dirPtX = m_posRobot.GetX() + 0.5 * cos(m_theta);
	float dirPtY = m_posRobot.GetY() + 0.5 * sin(m_theta);
	vector<int> dirPxl = CoordinateToPixel(dirPtX, dirPtY);
	Point dirPt(dirPxl[0], dirPxl[1]);
	line(m_imgrobot, Point(robotPixelX, robotPixelY), dirPt, CV_RGB(255, 125, 125), 2, 8);
	m_imgshow = m_imgmask + m_imgrobot;
}

vector<vector<string> >  Robot::GetGroundingInfo(string fileName)
{
	vector<vector<string> > res;
	fstream dataFile(fileName.c_str());
	string input;
	int t = 0;
	while( getline( dataFile, input ) ) 
	{
		cout << input << "\n";
		vector<string> command;
		string word;
		int i = 0;
		while (i < input.size())
		{
			if(input[i] == ',')
			{
				command.push_back(word);
				word = "";
				i += 2;
			}
			else
			{
				word.push_back(input[i]);
				i++;
			}
		}
		command.push_back(word);
		res.push_back(command);

		t++;
	}
	
	cout << "command size: " << res.size() << endl;
	m_groundings = res;
	return res;
}

void Robot::ReadGOInformation(string filename)
{
	cout << "running read information\n";

	ifstream infile;
	infile.open(filename.c_str());
	string line = "";
	vector<vector<string> > allTokens;
	
	while (getline(infile, line))
	{	
		//cout << line << endl;
		istringstream split(line);
		vector<string> tokens;
		char split_char = ' ';
		for (string each; getline(split, each, split_char); tokens.push_back(each));
		allTokens.push_back(tokens);
	}
	
	map<string, vector<float> > strToDimension;
	
	for (int k = 0; k < allTokens.size(); k++)
	{
		vector<string> tk = allTokens[k];
		for (int i = 0; i < tk.size(); i++)
		{
			if (tk[i].compare("define") == 0 && tk[i+2].compare("model") == 0)
			{	
				string category = tk[1];
				vector<float> dim(2, 0); 
				vector<string> tokens = allTokens[k+2];	
				//cout << category << " " << tokens[4] << " " << tokens[5] << endl;
				dim[0] = atof(tokens[4].c_str());
				dim[1] = atof(tokens[5].c_str());
				strToDimension[category] = dim;
			}
		}
	}
	
	for (int k = 0; k < allTokens.size(); k++)
	{
	  	int id;
		string category;
		string name;
		vector<float> pose;
		vector<string> tokens = allTokens[k];
		
		for (int i = 0; i < tokens.size(); i++)
		{	  
			if (tokens[i].compare("ranger_return") == 0)
			{
				id = (int)atof(tokens[i+1].c_str());
				category = tokens[0].substr( 0, tokens[0].size()-1 );
				name = tokens[9].substr( 1, tokens[9].size()-2 );
				float x = atof(tokens[3].c_str());
				float y = atof(tokens[4].c_str());
				float theta = atof(tokens[6].c_str());
				pose.push_back(x);
				pose.push_back(y);
				pose.push_back(theta);
				vector<float> dim = strToDimension[category];
				//cout << id << " " << category << " " << name << " " << pose[0] << " " << pose[1] << " " << pose[2] << " " << dim[0] << " " << dim[1] << endl;
				Fur f;
				f.name = name;
				vector<float> posedim(5, 0);
				posedim[0] = pose[0];
				posedim[1] = pose[1];
				posedim[2] = pose[2];
				if ((int)pose[2] == 180 || (int)pose[2] == 0 || (int)pose[2] < 0)
				{
					posedim[3] = dim[0];
					posedim[4] = dim[1];
				}
				else
				{
					posedim[3] = dim[1];
					posedim[4] = dim[0];
				}
				f.posedim = posedim;
				cout << "id:" << id << " name:" << f.name << " pose: " << f.posedim[0] << " " << f.posedim[1] << " " << f.posedim[2] << " dim: " << f.posedim[3] << " " << f.posedim[4] << endl;
				m_furLib[id] = f;
				break;
			}
		}		
	}

	infile.close();
	
}

#endif
