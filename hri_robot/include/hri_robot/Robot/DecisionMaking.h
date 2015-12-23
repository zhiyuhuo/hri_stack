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
// 	m_spatialCommand = "the mug is on the table on the right in the bedroom";
// 	AskGroundingService(m_spatialCommand);
// 	ShowRobotCmdInfo();

	while (ros::ok())
	{
		Test();
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
	if (m_mission.compare("init") == 0)
	{
// 		Perception();
// 		vector<float> originalRobotPose;
// 		originalRobotPose.push_back(m_posRobot.GetX());
// 		originalRobotPose.push_back(m_posRobot.GetY());
// 		originalRobotPose.push_back(m_theta);
// 		m_originalRobotPose = originalRobotPose;
// 		m_entities = GetVisiableEntities();
		SetOccupiedMap(400, 400, 0.25, -50, -50);
		VecPosition posTarget(2,-1);
		m_pathPoints.clear();
		m_pathPoints = CallForPathPlan(m_posRobot, posTarget);
		m_path = 0;
		m_mission = "run";
	}
	
	else if (m_mission.compare("run") == 0)
	{
		int r = ToPos(m_pathPoints[m_path]); 
		SetRobotVelocity();
		if (r == 1) 
		{
			cout << ++m_path << endl;
		}
		if (m_path > m_pathPoints.size())
		{
			m_mission = "stop";
		}
	}
	
	else if (m_mission.compare("stop") == 0)
	{
	  
	}
	
	
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
