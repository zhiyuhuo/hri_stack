#ifndef DECISIONMAKING_H
#define DECISIONMAKING_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <dirent.h>

#include "Header.h"

#include "hri_language_generation/GenerateSpatialLanguage.h"

int Robot::TestFromRDT(vector<string> groundings)
{
	BuildGroundingList(groundings);
	ShowRobotCmdInfo();
	FormatGroundings();
  
	while (ros::ok()) 
	{
		HomeFetchTask();
		ros::spinOnce();
	}
  
  
	return 0;	
}

int Robot::RunNode()
{
// 	m_spatialCommand = "the mug is on the table on the right in the bedroom";
// 	AskGroundingService(m_spatialCommand);
// 	ShowRobotCmdInfo();

/***For testing the navigation package.***/
// 	while (ros::ok())
// 	{
// 		TestPathGeneration();
// 		ros::spinOnce();
// 	}
  
/***For testing old decision making function.***/
// 	while (ros::ok())
// 	{
// 		DecisionMaking();
// 		ros::spinOnce();
// 	}
// 	BuildFakeGroundingList();

	while (ros::ok() && m_spatialCommand.size() == 0) 
	{
		ros::spinOnce();
	}
	AskGroundingService(m_spatialCommand);
	ShowRobotCmdInfo();
  
	while (ros::ok()) 
	{
		HomeFetchTask();
		ros::spinOnce();
	}
  
  
	return 0;
}

int Robot::HomeFetchTask()
{
	// cout << m_ifGetPerception << " " << m_ifGetPose << endl;
	// cout << "m_mission: " << m_mission << endl;
  	imshow("grid map", m_imgGrid);
	waitKey(1);
	
	if (m_mission == "init") 
	{
		CallForPercepstionService();
		if (m_ifGetPerception && m_ifGetPose) 
		{
			m_tempSEList.clear();
			m_mission = "wait_for_command";	
		}
	}
		
	else if (m_mission == "wait_for_command")
	{
		m_mission = "init_task";
	}
	  
	else if (m_mission == "init_task") 
	{
		m_posRobotLast = m_posRobot;
		m_action = "init";
		m_step = 0;
		
		if (m_groundings.size() <= 0)
		{
			exit(1);
		}
		
		vector<string> cmdVec = m_groundings[m_step];
		cout << "-----------------------------------------:\n";
		for (int i = 0; i < cmdVec.size(); i++)	
		{
			cout << cmdVec[i] << " ";	
		}	
		cout << endl;	
		m_mission = "follow_command";
	}

	else if (m_mission == "follow_command")
	{
		int r = 0;
		vector<string> cmdVec = m_groundings[m_step];
		
		if (cmdVec[0] != "non")
		{
			string worldAndroom = m_worldName + "_" + cmdVec[0];
			r = GotoRoom(worldAndroom);
		}
		else
		{
			r = RunCommand(cmdVec);
		}
		if (r == 1)
		{
			//cout << " m_pathLength: " << m_pathLength << endl;
			//cout << "Finished" << endl;
			cout << "Final Robot Pose For This RDT: " << m_posRobot.GetX() << ", " << m_posRobot.GetY() << ", " << m_theta*180/PI << endl;
			m_mission = "jump_to_next";
		}
		else if (r == -1)
		{
			cout << "Cannot run this command. Skip to the next\n";
			m_mission = "jump_to_next";
		}
	}
		
	else if (m_mission == "jump_to_next")
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
	
	else if (m_mission == "approach_the_furniture")
	{
		
	}
		
	else if (m_mission == "end")
	{
		m_mission = "end";
	}
		
	return 0;
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
	
	imshow("grid map", m_imgGrid);
	waitKey(1);
	m_pathLength += (m_posRobot - m_posRobotLast).GetMagnitude();
	m_posRobotLast = m_posRobot;
	return 0;	
}

int Robot::BatchTestSLG()
{
	string rootDir = "/home/hri/hri_DATA/language_generation_data/entities_log/";
	vector<string> entitiesListDirSet;
	DIR *dpdf;
	struct dirent *epdf;
	dpdf = opendir(rootDir.c_str());
	if (dpdf != NULL)
	{
		while (epdf = readdir(dpdf))
		{  
			//std::std::cout << epdf->d_name << endl;
			string dirName(epdf->d_name);
			if (dirName.size() > 5)
			{
				entitiesListDirSet.push_back(rootDir + dirName);
			}
		}
	}
	
	for (int i = 0; i < entitiesListDirSet.size(); i++)
	{
		cout << entitiesListDirSet[i] << endl;
	}
	return 0;
}

int Robot::Test()
{
	
	return 0;
}

int Robot::TestPathGeneration()
{
	if (m_mission.compare("init") == 0)
	{
		CallForPercepstionService();
		if (m_ifGetPerception && m_ifGetPose) 
		{
			m_mission = "path_plan";	
		}
	}
	
	else if (m_mission.compare("path_plan") == 0)
	{
		//SetOccupancyMap(OCCUPANCYMAP_W, OCCUPANCYMAP_H, OCCUPANCYMAP_RL, OCCUPANCYMAP_X, OCCUPANCYMAP_Y);
		VecPosition posTarget(3,-0.5);
		m_pathPoints.clear();
		m_pathPoints = CallForPathPlan(m_posRobot, posTarget);
// 		for (int i = 0; i < m_pathPoints.size(); i++)
// 		{
// 		      cout << m_pathPoints[i].GetX() << " " << m_pathPoints[i].GetY() << endl;
// 		}
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
		if (m_path >= m_pathPoints.size())
		{
			m_mission = "stop";
		}
	}
	
	else if (m_mission.compare("stop") == 0)
	{
		m_linearSpeed = 0;
		m_angularSpeed = 0;
		SetRobotVelocity();
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

int Robot::KeyboardControlForLanguageGeneration(string worldName, string targetObject)
{
	int res = 0;
	//m_targetObject = "mug";
	
	while (ros::ok())
	{
		imshow("grid map", m_imgGrid);
		char c = waitKey(1);
        
		if (m_mission == "init") 
		{
			m_LEList.clear();
			m_worldName = worldName;
			m_targetObject = targetObject;
			string LEDir("/home/hri/hri_DATA/LE_" + worldName);
			cout << "loading le from " << LEDir << endl;
			ImportLEList(LEDir.c_str());
			CallForPercepstionService();
			if (m_ifGetPerception && m_ifGetPose) 
			{
			    m_tempSEList.clear();
			    m_mission = "get_first_perception";	
			}
		}

		if (m_mission.compare("get_first_perception") == 0)
		{
			cout << "real robot pose: " << m_posRobot.GetX() << "    " << m_posRobot.GetY() << "    " << m_theta << endl; 
			CallForPercepstionService();
			Perception();
			m_entities = GetVisiableEntities();
			//SetOccupancyMap(OCCUPANCYMAP_W, OCCUPANCYMAP_H, OCCUPANCYMAP_RL, OCCUPANCYMAP_X, OCCUPANCYMAP_Y);
            
			m_mission = "receive_keyboard_action";
		}
        
		else if (m_mission.compare("receive_keyboard_action") == 0)
		{
			switch (c)
			{
				case 'w':	
				{	
					m_linearSpeed = 0.1; 
					m_angularSpeed = 0;	
					break;
				}
				case 'a':	
				{	
					m_angularSpeed = (m_linearSpeed >= 0? 1:-1) * 0.2;	 
					break;
				}
				case 'd':	
				{			  
					m_angularSpeed = (m_linearSpeed >= 0? 1:-1)*(-0.2);	 
					break;
				  
				}
				case 's':	
				{	
					m_linearSpeed = 0; 
					m_angularSpeed = 0;		
					break;
				}
				case 'x':
				{	
					m_linearSpeed = -0.1; 
					m_angularSpeed = 0;	 
					break;			  
				}
				case 'c':
				{	
					cout << "real robot pose: " << m_posRobot.GetX() << "    " << m_posRobot.GetY() << "    " << m_theta << endl; 
					CallForPercepstionService();
					Perception();
					m_entities = GetVisiableEntities();
					SetOccupancyMap(OCCUPANCYMAP_W, OCCUPANCYMAP_H, OCCUPANCYMAP_RL, OCCUPANCYMAP_X, OCCUPANCYMAP_Y);
					DrawOccupancyGrid();
                    
					cout << "The entities in the map: " << endl;
					for (int i = 0; i < m_entities.size(); i++)
					{
						cout << " -" << m_entities[i].name << ": " << m_entities[i].vec.size()/2 << ", " << m_entities[i].x << ", " << m_entities[i].y << ",   " << m_entities[i].dir << endl;
 					}
					break;			  
				}
                
				case 'l':
				{
					//AnalyseEntityRelation();
					//break;
					string entitiesFileDir = "/home/hri/hri_DATA/language_generation_data/entities_log/" + m_worldName + "_" + m_targetObject + ".txt";
				        SaveEntitiesInformationToTXT(entitiesFileDir, m_entities);
					//ReadEntitiesInformationFromTXT(entitiesFileDir);
					break;
					
					map<string, vector<Dct> > dcts = LoadGroundingTypesList("/home/hri/hri_DATA/Targets/");
					vector<string> dscpSet = GenerateStaticDescription(dcts);		
					dscpSet = AdjustGroundingsFormatToLGServer(dscpSet); // temp add here need to be removed later
					    
					hri_language_generation::GenerateSpatialLanguage srv;
					for (int i = 0; i < dscpSet.size(); i++)
					{
					    cout << i << ": " << dscpSet[i] << endl;
					    srv.request.groundings.push_back(dscpSet[i]);
					}
					  
					if (m_generatingLanguageClient.call(srv))
					{
					    cout << srv.response.language << endl;
					}
					else
					{
					    ROS_ERROR("Failed to call service SpatialLanguageGrounding\nLet's try it again.");
					}
    
					break;
					
				}
			}
		}	
		SetRobotVelocity();
		
		ros::spinOnce();
	}
	return res;
}

int Robot::AutomaticLanguageGenerationFromVideo(string worldName, string targetObject)
{
	int res = 0;
	m_targetObject = targetObject;
	
	while (ros::ok())
	{
		imshow("grid map", m_imgGrid);
		char c = waitKey(30);
        
		if (m_mission == "init") 
		{
			m_LEList.clear();
			m_worldName = worldName;
			m_targetObject = targetObject;
			string LEDir("/home/hri/hri_DATA/LE_" + worldName);
			cout << "loading le from " << LEDir << endl;
			ImportLEList(LEDir.c_str());
			CallForPercepstionService();
			if (m_ifGetPerception && m_ifGetPose) 
			{
			    m_tempSEList.clear();
			    m_mission = "get_first_perception";	
			}
		}

		if (m_mission.compare("get_first_perception") == 0)
		{
			cout << "real robot pose: " << m_posRobot.GetX() << "    " << m_posRobot.GetY() << "    " << m_theta << endl; 
			CallForPercepstionService();
			Perception();
			m_entities = GetVisiableEntities();
			SetOccupancyMap(OCCUPANCYMAP_W, OCCUPANCYMAP_H, OCCUPANCYMAP_RL, OCCUPANCYMAP_X, OCCUPANCYMAP_Y);
            
			m_mission = "receive_keyboard_action";
		}
        
		else if (m_mission.compare("receive_keyboard_action") == 0)
		{
			switch (c)
			{
				case 'l':
				{
					//AnalyseEntityRelation();
					//break;
				      
					map<string, vector<Dct> > dcts = LoadGroundingTypesList("/home/hri/hri_DATA/Targets/");
					vector<string> dscpSet = GenerateStaticDescription(dcts);		
					dscpSet = AdjustGroundingsFormatToLGServer(dscpSet); // temp add here need to be removed later
					    
					hri_language_generation::GenerateSpatialLanguage srv;
					for (int i = 0; i < dscpSet.size(); i++)
					{
					    cout << i << ": " << dscpSet[i] << endl;
					    srv.request.groundings.push_back(dscpSet[i]);
					}
					  
					if (m_generatingLanguageClient.call(srv))
					{
					    cout << srv.response.language << endl;
					}
					else
					{
					    ROS_ERROR("Failed to call service SpatialLanguageGrounding\nLet's try it again.");
					}
    
					break;
					
				}
				default:
				{	
					cout << "real robot pose: " << m_posRobot.GetX() << "    " << m_posRobot.GetY() << "    " << m_theta << endl; 
					CallForPercepstionService();
					Perception();
					m_entities = GetVisiableEntities();
					SetOccupancyMap(OCCUPANCYMAP_W, OCCUPANCYMAP_H, OCCUPANCYMAP_RL, OCCUPANCYMAP_X, OCCUPANCYMAP_Y);
					DrawOccupancyGrid();
                    
					cout << "The entities in the map: " << endl;
					for (int i = 0; i < m_entities.size(); i++)
					{
						cout << " -" << m_entities[i].name << ": " << m_entities[i].vec.size()/2 << ", " << m_entities[i].x << ", " << m_entities[i].y
						     << ",   " << m_entities[i].dir << ",   " << m_entities[i].confidence << endl;
 					}
					break;			  
				}
			}
		}	
		
		ros::spinOnce();
	}
	return res;	
}


#endif
