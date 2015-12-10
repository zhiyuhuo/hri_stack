#ifndef DECISIONMAKING_H
#define DECISIONMAKING_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include <cmath>
#include <fstream>

#include "Header.h"

int Robot::RunNode()
{
	m_spatialCommand = "the mug is on the table on the right in the bedroom";
	AskGroundingService(m_spatialCommand);
	ShowRobotCmdInfo();
	geometry_msgs::Twist msg;
	msg.angular.z = 0.1;
	m_speedPub.publish(msg);
	while (ros::ok())
	{
	  	msg.angular.z = 0.1;
		m_speedPub.publish(msg);
		ros::spinOnce();
	}
}

int Robot::DecisionMaking()
{
	m_targetRoom = "livingroom";
	m_targetObject = "mug";
	
	if (m_mission.compare("init") == 0)
	{
		m_posRobotLast = m_posRobot;
		m_action = "init";
		m_step = 0;
		
		vector<string> cmdVec = m_groundings[m_step];
		cout << "-----------------------------------------:\n";
		for (int i = 0; i < cmdVec.size(); i++)	{	cout << cmdVec[i] << " ";	}	cout << endl;
		
		m_mission = "follow_command";
	}
	
	else if (m_mission.compare("follow_command") == 0)
	{
		int r = 0;
		vector<string> cmdVec = m_groundings[m_step];
		r = RunCommand(cmdVec);
		if (r == 1)
		{
			//cout << " m_pathLength: " << m_pathLength << endl;
			//cout << "Finished" << endl;
			cout << "Final Robot Pose For This RDT: " << m_posRobot.GetX() << ", " << m_posRobot.GetY() << ", " << m_theta*180/PI << endl;
			m_mission = "jump_to_next";
		}
	}
	
	else if (m_mission.compare("jump_to_next") == 0)
	{
		if (m_step == m_groundings.size()-1)
		{
			cout << "mission finished pose: " << m_posRobot.GetX() << ", " << m_posRobot.GetY() << ", " << m_theta*180/PI << endl;
			cout << "mission accomplished!" << endl;
			m_mission = "end";
		}
		else
		{
		      m_step++;	
		      m_move = "init";
		      m_action = "init";
		      m_state = "init";
		      m_currentDct.clear();
		      m_currentScore = 0;
		      m_entities.clear();
		      
		      vector<string> cmdVec = m_groundings[m_step];
		      for (int i = 0; i < cmdVec.size(); i++)	{	cout << cmdVec[i] << " ";	}	cout << endl;
		      m_mission = "follow_command";
		}
	}
	
	else if (m_mission.compare("end") == 0)
	{
		m_mission = "end";
	}
	
	imshow("grid map", m_imgOccupancy);
	waitKey(1);
	m_pathLength += (m_posRobot - m_posRobotLast).GetMagnitude();
	m_posRobotLast = m_posRobot;
	return 0;	
}

int Robot::Test()
{
	m_targetRoom = "livingroom";
  	vector<string> cmdVec;
	cmdVec.push_back("non");
	cmdVec.push_back("non");
	cmdVec.push_back("wall");
	cmdVec.push_back("front");
	cmdVec.push_back("non");
	
	if (m_mission.compare("init") == 0)
	{
		m_mission = "search";
		m_posRobotLast = m_posRobot;
		m_action = "init";
	}
	
	else if (m_mission.compare("search") == 0)
	{
		int r = 1;
		//r = SearchSpin();
		if (r == 1)
		{
			//cout << " m_pathLength: " << m_pathLength << endl;
			m_mission = "follow_command";
		}
	}
	
	else if (m_mission.compare("follow_command") == 0)
	{
		int r = 0;
		//r = FollowCommand(cmdVec);
		r = RunCommand(cmdVec);
		if (r == 1)
		{
			//cout << " m_pathLength: " << m_pathLength << endl;
			cout << "Finished" << endl;
			cout << "Final Robot Pose: " << m_posRobot.GetX() << ", " << m_posRobot.GetY() << ", " << m_theta*180/PI << endl;
			m_mission = "end";
		}
	}
	
	else if (m_mission.compare("end") == 0)
	{
		exit(1);
		m_mission = "end";
	}
	
	m_pathLength += (m_posRobot - m_posRobotLast).GetMagnitude();
	m_posRobotLast = m_posRobot;
	return 0;
}

int Robot::TestRead()
{
	m_targetRoom = "livingroom";
  	vector<string> cmdVec;
	cmdVec.push_back("non");
	cmdVec.push_back("non");
	cmdVec.push_back("move");
	cmdVec.push_back("front");
	cmdVec.push_back("non");
	
	vector<Dct> dctSet = ReadDetector(cmdVec);
	
	return 0;
}


#endif
