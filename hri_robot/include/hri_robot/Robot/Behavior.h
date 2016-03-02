#ifndef ROBOTBEHAVIOR_H
#define ROBOTBEHAVIOR_H

#include "stdio.h"
#include "stdlib.h"
#include <cstdlib>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <dirent.h>

#include "Header.h"
using namespace std;

int Robot::RunCommand(vector<string> cmd)
{
	int res = 0;
	
	if (m_state.compare("init") == 0)
	{
		CallForPercepstionService();
		Perception();
		vector<float> originalRobotPose;
		originalRobotPose.push_back(m_posRobot.GetX());
		originalRobotPose.push_back(m_posRobot.GetY());
		originalRobotPose.push_back(m_theta);
		m_originalRobotPose = originalRobotPose;
		m_currentScore = 0;
		m_state = "read_dct";
	}
	
	else if (m_state.compare("read_dct") == 0)
	{
		m_currentDct = ReadDetector(cmd);
		m_state = "navigation";
	}
	
	else if (m_state.compare("navigation") == 0)
	{
		m_entities = GetVisiableEntities();
		m_currentScore = ScoreCurrentState(m_currentDct);
		cout << "m_currentScore = " << m_currentScore << endl;

		if (m_currentScore > 0.7)
		{
			m_state = "end";
		}
		else
		{
			float r2 = IterateltSearchTarget(m_currentDct);
			cout << "r2 = " << r2 << endl;
			if (r2 < 0)
			{
				m_state = "search";
			}
			else
			{
				if (r2 > m_currentScore || r2 > 0.7)
				{
					m_state = "plan_path";
// 					m_state = "directly_to_targert";
					cout << "m_moveTarget: " << m_moveTarget.GetX() << " " << m_moveTarget.GetY() << " " << m_turnTarget << endl;
				}
				else
				{
					m_state = "end";
				}
			}
		
		}
		
		cout << "State to Move: " << m_state << endl;
	}
	
	else if (m_state.compare("search") == 0)
	{
		int r = Search90();
		if (r == 1)
		{
			m_state = "navigation";
		}
	}
	
	else if (m_state.compare("plan_path") == 0)
	{
		CallForPercepstionService();
		Perception();
		m_entities = GetVisiableEntities();
		SetOccupiedMap(400, 400, 0.25, -50, -50);
		m_pathPoints.clear();
		m_pathPoints = CallForPathPlan(m_posRobot, m_moveTarget);
		m_path = 0;
		m_state = "move_to_targert";
	}
	
	else if (m_state.compare("move_to_targert") == 0)
	{
		if (m_path < m_pathPoints.size())
		{
			if (ToPos(m_pathPoints[m_path])) 
			{
				cout << ++m_path << endl;
			}
			SetRobotVelocity();
		}
		else if (m_path == m_pathPoints.size())
		{
			if (ToAngle(m_turnTarget)) 
			{
				cout << ++m_path << endl;
			}
			SetRobotVelocity();
		}
		else if (m_path > m_pathPoints.size())
		{	
			m_state = "end";
		}
	}
	
	else if (m_state.compare("directly_to_targert") == 0)
	{
		if (ToPosAngle(m_moveTarget, m_turnTarget) > 0) 
		{
			m_state = "end";
		}
		SetRobotVelocity();
	}
	
	else if (m_state.compare("end") == 0)
	{
		m_linearSpeed = 0;
		m_angularSpeed = 0;
		SetRobotVelocity();
		res = 1;
	}
	
	return res;
}

int Robot::GotoRoom(string room)
{
	int res = 0;
	return res;
}

int Robot::RDTIndoorNavigation(RDTNode rdt)
{
	int res = 0;
	return res;
}

int Robot::Search90()
{
	int res = 0;
	float rotc = PI / 4;
	if (m_action.compare("init") == 0)
	{
	  	CallForPercepstionService();
		Perception();
		m_turnTarget = m_theta - rotc;
		m_action = "search_-45";
		cout << m_action << endl;
	}
	
	else if (m_action.compare("search_-45") == 0)
	{
		if (ToAngle(m_turnTarget) == 1)
		{
			CallForPercepstionService();
			Perception();
			m_turnTarget = m_theta - rotc;
			m_action = "search_-90";
			cout << m_action << endl;
		}
	}
	
	else if (m_action.compare("search_-90") == 0)
	{
		if (ToAngle(m_turnTarget) == 1)
		{
			CallForPercepstionService();
			Perception();
			m_turnTarget = m_theta + rotc * 3;
			m_action = "search_45";
			cout << m_action << endl;
		}
	}
	
	else if (m_action.compare("search_45") == 0)
	{
		if (ToAngle(m_turnTarget) == 1)
		{
			CallForPercepstionService();
			Perception();
			m_turnTarget = m_theta + rotc;
			m_action = "search_90";
			cout << m_action << endl;
		}
	}
	
	else if (m_action.compare("search_90") == 0)
	{
		if (ToAngle(m_turnTarget) == 1)
		{
			CallForPercepstionService();
			Perception();
			m_turnTarget = m_theta - 2*rotc;
			m_action = "move_back";
			cout << m_action << endl;
		}
	}
	
	else if (m_action.compare("move_back") == 0)
	{
		if (ToAngle(m_turnTarget) == 1)
		{
			m_action = "stop";
			cout << m_action << endl;
		}
	}
	
	else if (m_action.compare("stop") == 0)
	{
		res = 1;
	}
	
	return res;
}

vector<Dct> Robot::ReadDetector(vector<string> cmd)
{
	vector<Dct> res;
	string cmdStr = cmd[0] + "_" + cmd[1] + "_" + cmd[2] + "_" + cmd[3] + "_" + cmd[4];
	string directory = "/home/hri/hri_DATA/Targets/" + cmdStr + "/";
	vector<string> fileList;
	
	DIR *dpdf;
	struct dirent *epdf;
	dpdf = opendir(directory.c_str());
	if (dpdf != NULL)
	{
		while (epdf = readdir(dpdf))
		{  
			//cout << epdf->d_name << endl;
			string fileName(epdf->d_name);
			if (fileName.size() > 5)
			{
				fileList.push_back(directory + fileName);
				string featureName = fileName.substr(0, fileName.size() - 4);
				cout << "required sp name:" << featureName << endl;
			}
		}
	}
	
	for (int i = 0; i < fileList.size(); i++)
	{
		cout << "flname: " << fileList[i] << endl;
		string sLine = "";
		ifstream infile;
		infile.open(fileList[i].c_str());
		getline(infile, sLine);
		vector<float> outdirw;
		vector<float> indirw;
		vector<float> distw;

		Dct dct;
		char c = 0;
		string str;
		int stop = 0;
		for (int k = 0; k < sLine.size(); k++)
		{
			c = sLine[k];
			if (c == '_')
			{
				stop = k;
				break;
			}
		}
		string nameA = sLine.substr(0, stop);
		string nameB = sLine.substr(stop+1, sLine.size() - stop - 2);
		//cout << "-" << nameA << "_" << nameB << "-" << endl;
		getline(infile, sLine);
		for (int k = 0; k < sLine.size(); k++)
		{
			c = sLine[k];
			if (c >= 48 && c <= 57 || c == '.' || c == '-')
			{
				str.push_back(c);
			}
			else
			{
				if (str.size() > 0)
				{
					//cout << str << " ";
					outdirw.push_back(atof(str.c_str()));
					str = "";
				}
			}
		}
		//cout << endl;
		//cout << outdirw.size() << endl;
		
		sLine = "";
		getline(infile, sLine);
		for (int k = 0; k < sLine.size(); k++)
		{
			c = sLine[k];
			if (c >= 48 && c <= 57 || c == '.' || c == '-')
			{
				str.push_back(c);
			}
			else
			{
				if (str.size() > 0)
				{	
					//cout << str << " ";
					indirw.push_back(atof(str.c_str()));
					str = "";
				}
			}
		}
		//cout << endl;
		//cout << indirw.size() << endl;
		
		sLine = "";
		getline(infile, sLine);
		for (int k = 0; k < sLine.size(); k++)
		{
			c = sLine[k];
			if (c >= 48 && c <= 57 || c == '.' || c == '-')
			{
				str.push_back(c);
			}
			else
			{
				if (str.size() > 0)
				{	
					//cout << str << " ";
					distw.push_back(atof(str.c_str()));
					str = "";
				}
			}
		}
		//cout << endl;
		//cout << distw.size() << endl;	
		
		dct.nameA = nameA;
		dct.nameB = nameB;
		dct.outdirw = outdirw;
		dct.indirw = indirw;
		dct.distw = distw;
		res.push_back(dct);
		
		infile.close();
	}
	
	return res;
}

float Robot::ScoreCurrentState(vector<Dct> decisionSpatialRelations)
{
	float res = 1;
	float res2 = 0;
	//decide if there is enough entities to make decision
	vector<string> requiredEntitiesNames;
	for (int i = 0; i < decisionSpatialRelations.size(); i++)
	{
		string nameA = decisionSpatialRelations[i].nameA;
		string nameB = decisionSpatialRelations[i].nameB;
		requiredEntitiesNames.push_back(nameA);
		requiredEntitiesNames.push_back(nameB);
	}  
	  
	vector<string> myEntitiesNames;
	myEntitiesNames.push_back("rotation");
	for (int i = 0; i < m_entities.size(); i++)
	{
// 		cout << "I have " << m_entities[i].name << endl;
		myEntitiesNames.push_back(m_entities[i].name);
	}
	
	vector<int> matchVector(requiredEntitiesNames.size(), 0);
	for (int i = 0; i < requiredEntitiesNames.size(); i++)
	{
// 		cout << "I need " << requiredEntitiesNames[i] << endl;
		for (int j = 0; j < myEntitiesNames.size(); j++)
		{
			if (requiredEntitiesNames[i].compare(myEntitiesNames[j]) == 0)
			{
				matchVector[i] = 1;
				break;
			}
		}
	}
	
	for (int i = 0; i < matchVector.size(); i++)
	{
		if (matchVector[i] == 0)
		{
			cout << "entity " << requiredEntitiesNames[i] << " is not observed" << endl;
			res = 0;
			return res;
		}
	}
							  
	SpR rot;
	rot.nameA = "rotation";
	rot.nameB = "rotation";
	float rotationf = m_theta - m_originalRobotPose[2];
	while (rotationf > PI)	{	rotationf -= 2*PI;	}
	while (rotationf < -PI)	{	rotationf += 2*PI;	}
	vector<float> rotoutdirw(1, rotationf);
	vector<float> rotindirw(1, 0);
	rot.outdirw = rotoutdirw;
	rot.indirw = rotindirw;
	
	for (int i = 0; i < m_entities.size(); i++)
	{
		if (m_entities[i].name.compare("CR") == 0)
		{
			m_entities[i] = GetRobotEntity( m_posRobot.GetX(), m_posRobot.GetY(), m_theta );
			m_entities[i].name = "CR";
		}
	}
	cout << m_posRobot.GetX() << ", " << m_posRobot.GetY() << ", " << m_theta << endl;
	
	float respmin = 1;
	for (int i = 0; i < decisionSpatialRelations.size(); i++)
	{
		Dct dct = decisionSpatialRelations[i];
		SpR spAB;
		if (dct.nameA.find("rotation") != string::npos  ||  dct.nameB.find("rotation") != string::npos)
		{
			spAB = rot;
		}
		else
		{
			Ent A;
			Ent B;		
			for (int j = 0; j < m_entities.size(); j++)
			{
				if (m_entities[j].name.compare(dct.nameA) == 0)
				{
					A = m_entities[j];
				}
				if (m_entities[j].name.compare(dct.nameB) == 0)
				{
					B = m_entities[j];
				}
			}	
			cout << A.name << " " << A.vec.size()/2 << ", " << B.name << " " << B.vec.size()/2 << endl;
			spAB = GetSpatialRelationB2A(A, B);			
			
		}
		
		float dctResponse = GetResponseOfADetector(spAB, dct);
		cout << dct.nameA << "_" << dct.nameB << ", " << dctResponse << endl;

		if (dctResponse < respmin)
		{
			respmin = dctResponse;
		}
	}
	
	return respmin;
}

float Robot::IterateltSearchTarget(vector<Dct> decisionSpatialRelations)
{
	float res = 1;
	VecPosition posRobot = m_posRobot;
	//decide if there is enough entities to make decision
	vector<string> requiredEntitiesNames;
	for (int i = 0; i < decisionSpatialRelations.size(); i++)
	{
		string nameA = decisionSpatialRelations[i].nameA;
		string nameB = decisionSpatialRelations[i].nameB;
		
		if (nameA.compare("rotation") != 0 && nameA.compare("rotation") != 0)
		{
			requiredEntitiesNames.push_back(nameA);
			requiredEntitiesNames.push_back(nameB);
		}
	}  
	
	sort(requiredEntitiesNames.begin(), requiredEntitiesNames.end());
        requiredEntitiesNames.erase(unique(requiredEntitiesNames.begin(), requiredEntitiesNames.end()), requiredEntitiesNames.end());
	  
	vector<string> myEntitiesNames;
	for (int i = 0; i < m_entities.size(); i++)
	{
		myEntitiesNames.push_back(m_entities[i].name);
	}
	
	vector<int> matchVector(requiredEntitiesNames.size(), 0);
	for (int i = 0; i < requiredEntitiesNames.size(); i++)
	{
		for (int j = 0; j < myEntitiesNames.size(); j++)
		{
			if (requiredEntitiesNames[i].compare(myEntitiesNames[j]) == 0)
			{
				matchVector[i] = 1;
				break;
			}
		}
	}
	
	for (int i = 0; i < matchVector.size(); i++)
	{
		int kk = matchVector[i];
		if (matchVector[i] == 0)
		{
			cout << "entity " << requiredEntitiesNames[i] << " is not observed" << endl;
			res = -1;
			return res;
		}
	}
	
	if (res > 0)
	{
		//First Step - Calculate Position by 1m & 1/4PI resolution
	  	vector<vector<Ent> > candidatedEnts = ListEntitiesSetCombination(requiredEntitiesNames, m_entities);
	  
	  	float score = 0;
		vector<float> finalTargetPose(3, 0);
		
		float score1 = 0;
		vector<float> respset;
		vector<float> movePose(3, 0);
		
		for (int E = 0; E < candidatedEnts.size(); E++)
		{		
			vector<Ent> entities = candidatedEnts[E];
			vector<vector<float> > xythSet;
			
			for ( float x = -4 + posRobot.GetX(); x <= 4 + posRobot.GetX(); x+=0.5 )
			{
				for ( float y = -4 + posRobot.GetY(); y <= 4 + posRobot.GetY(); y+=0.5 )
				{
					for ( float th = 0; th < PI * 2; th += PI / 4 )
					{
						vector<float> xyth(3, 0);
						xyth[0] = x;
						xyth[1] = y;
						xyth[2] = th;
						xythSet.push_back(xyth);
					}
				}
			}
			
			for (int i = 0; i < xythSet.size(); i++)
			{
			
// 				cout << x << " " << y  << " " << th << endl;	
				vector<float> xyth = xythSet[i];
				float x = xyth[0];
				float y = xyth[1];
				float th = xyth[2];
				SpR sprot;
				sprot.nameA = "rotation";
				sprot.nameB = "rotation";
				float rotationf = th - m_originalRobotPose[2];
				while (rotationf > PI)	{	rotationf -= 2*PI;	}
				while (rotationf < -PI)	{	rotationf += 2*PI;	}
				vector<float> rotoutdirw(1, rotationf);
				vector<float> rotindirw(1, 0);
				sprot.outdirw = rotoutdirw;
				sprot.indirw = rotindirw;
						
				for (int i = 0; i < entities.size(); i++)
				{
					if (entities[i].name.compare("CR") == 0)
					{
						entities[i] = GetRobotEntity( x, y, th );
						entities[i].name = "CR";
					}
				}
						
				float respadd = 0;
				vector<float> respbbb;
				for (int i = 0; i < decisionSpatialRelations.size(); i++)
				{
					Dct dct = decisionSpatialRelations[i];
					SpR spAB;
					if (dct.nameA.find("rotation") != string::npos  ||  dct.nameB.find("rotation") != string::npos)
					{
						spAB = sprot;
					}
					else
					{
						Ent A;
						Ent B;		
						for (int j = 0; j < entities.size(); j++)
						{
							if (entities[j].name.compare(dct.nameA) == 0)
							{
								A = entities[j];
							}
							if (entities[j].name.compare(dct.nameB) == 0)
							{
								B = entities[j];
							}
						}										
						spAB = GetSpatialRelationB2A(A, B);			
					}
					
					float dctResponse = GetResponseOfADetector(spAB, dct);
					//cout << dct.nameA << "_" << dct.nameB << ", " << dctResponse << endl;
					respadd += dctResponse;
					respbbb.push_back(dctResponse);
				}
						
				respadd /= decisionSpatialRelations.size();
				//cout << respadd << endl;
						
				if (respadd > score1)
				{
					int xi = (int)(x / 0.1) + 100;
					int yi = (int)(y / 0.1) + 100;
					if (m_imgOccupancy.data[xi + yi*200] == 0)
					{
						respset = respbbb;
						score1 = respadd;
						movePose[0] = x;
						movePose[1] = y;
						movePose[2] = th;
					}
				}

			}
			score = score1;
			for (int i = 0; i < respset.size(); i++) {	cout << respset[i] << " ";	}	cout << endl;
			cout << "move to pose1: " << movePose[0] << " " << movePose[1] << " " << movePose[2] << ", score1: " << score1 << endl;
			
			float score2 = 0;
			vector<float> movePose2 = movePose;
			for ( float x = -1 + movePose[0]; x <= 1 + movePose[0]; x += 0.25 )
			{
				for ( float y = -1 + movePose[1]; y <= 1 + movePose[1]; y += 0.25 )
				{
					for ( float th = 0; th < PI * 2; th += PI / 8 )
					{
// 						cout << " <" << x << " " << y  << " " << th << "> ";
						SpR sprot;
						sprot.nameA = "rotation";
						sprot.nameB = "rotation";
						float rotationf = th - m_originalRobotPose[2];
						while (rotationf > PI)	{	rotationf -= 2*PI;	}
						while (rotationf < -PI)	{	rotationf += 2*PI;	}
						vector<float> rotoutdirw(1, rotationf);
						vector<float> rotindirw(1, 0);
						sprot.outdirw = rotoutdirw;
						sprot.indirw = rotindirw;
						
						for (int i = 0; i < entities.size(); i++)
						{
							if (entities[i].name.compare("CR") == 0)
							{
								entities[i] = GetRobotEntity( x, y, th );
								entities[i].name = "CR";
							}
						}
						
							
						float respadd = 0;
						vector<float> respbbb;
						for (int i = 0; i < decisionSpatialRelations.size(); i++)
						{
							Dct dct = decisionSpatialRelations[i];
							SpR spAB;
							if (dct.nameA.find("rotation") != string::npos  ||  dct.nameB.find("rotation") != string::npos)
							{
								spAB = sprot;
							}
							else
							{
								Ent A;
								Ent B;		
								for (int j = 0; j < entities.size(); j++)
								{
									if (entities[j].name.compare(dct.nameA) == 0)
									{
										A = entities[j];
									}
									if (entities[j].name.compare(dct.nameB) == 0)
									{
										B = entities[j];
									}
								}										
								spAB = GetSpatialRelationB2A(A, B);			
							}
								
							float dctResponse = GetResponseOfADetector(spAB, dct);
							//cout << dct.nameA << "_" << dct.nameB << ", " << dctResponse << endl;
							respadd += dctResponse;
							respbbb.push_back(dctResponse);
						}
						
						respadd /= decisionSpatialRelations.size();
						//cout << respadd << endl;
						
						if (respadd > score2)
						{
							int xi = (int)(x / 0.1) + 100;
							int yi = (int)(y / 0.1) + 100;
							if (m_imgOccupancy.data[xi + yi*200] == 0)
							{
								respset = respbbb;
								score1 = respadd;
								movePose2[0] = x;
								movePose2[1] = y;
								movePose2[2] = th;
							}
						}					
					}
				}
				
			}			
			for (int i = 0; i < respset.size(); i++) {	cout << respset[i] << " ";	}	cout << endl;
			cout << "move to pose2: " << movePose2[0] << " " << movePose2[1] << " " << movePose2[2] << ", score2: " << score2 << endl;
		
			if (score2 > score)
			{
				score = score2;
				finalTargetPose[0] = movePose2[0];
				finalTargetPose[1] = movePose2[1];
				finalTargetPose[2] = movePose2[2];
			}
			else
			{
				finalTargetPose[0] = movePose[0];
				finalTargetPose[1] = movePose[1];
				finalTargetPose[2] = movePose[2];
			}
		}
		
		cout << "move to pose final: " << finalTargetPose[0] << " " << finalTargetPose[1] << " " << finalTargetPose[2] << ", final score: " << score << endl;
		
		m_moveTarget.SetX(finalTargetPose[0]);
		m_moveTarget.SetY(finalTargetPose[1]);
		m_turnTarget = finalTargetPose[2];
		
		res = 1;
		
		//exit(0);
	}
	
	return res;	
}

vector<vector<Ent> > Robot::ListEntitiesSetCombination(vector<string> requiredEntsNames, vector<Ent> myEnts)
{
	vector<vector<Ent> > res;
	
// 	for (int i = 0; i < requiredEntsNames.size(); i++)
// 	{
// 		cout << requiredEntsNames[i] << " ";
// 	}cout << endl;
// 	
// 	for (int i = 0; i < myEnts.size(); i++)
// 	{
// 		cout << myEnts[i].name << " ";
// 	}cout << endl;
	
	vector<Ent> validEnts;
	for (int i = 0; i < myEnts.size(); i++)
	{
		for (int j = 0; j < requiredEntsNames.size(); j++)
		{
			if (myEnts[i].name.compare(requiredEntsNames[j]) == 0)
			{
				validEnts.push_back(myEnts[i]);
				break;
			}
		}
	}
	
// 	for (int i = 0; i < validEnts.size(); i++)
// 	{
// 		cout << validEnts[i].name << " ";
// 	}cout << endl;
	
	vector<vector<int> > InstancesGroups;	
	for (int i = 0; i < requiredEntsNames.size(); i++)
	{
		vector<int> instancelist;
		InstancesGroups.push_back(instancelist);
	}
	
	int maxInst = 0;
	for (int i = 0; i < requiredEntsNames.size(); i++)
	{
		for (int j = 0; j < validEnts.size(); j++)
		{
			if (validEnts[j].name.compare(requiredEntsNames[i]) == 0)
			{
				InstancesGroups[i].push_back(j);
			}
		}
	}
	
	for (int i = 0; i < InstancesGroups.size(); i++)
	{
		cout << requiredEntsNames[i] << ": ";
		vector<int> instancelist = InstancesGroups[i];
// 		for (int j = 0; j < instancelist.size(); j++)
// 		{
// 			cout << instancelist[j] << " ";
// 		}cout << endl;
	}
	
	vector<int> singleEnts;
	vector<vector<int> > multiEntsSet;
	vector<vector<int> > combination;
	for (int i = 0; i < InstancesGroups.size(); i++)
	{
		if (InstancesGroups[i].size() == 1)
		{
			singleEnts.push_back(InstancesGroups[i][0]);
		}
		else
		{
			multiEntsSet.push_back(InstancesGroups[i]);
		}
	}
	vector<int> baseEnt = singleEnts;
	
	if (multiEntsSet.size() > 0)
	{	
		cout << "all the multiple combine--------------\n";
		for (int n0 = 0; n0 < multiEntsSet[0].size(); n0++)
		{
			baseEnt = singleEnts;
			if (multiEntsSet.size() > 1)
			{
				for (int n1 = 0; n1 < multiEntsSet[1].size(); n1++)
				{
					if (multiEntsSet.size() > 2)
					{
						for (int n2 = 0; n2 < multiEntsSet[2].size(); n2++)
						{
							if (multiEntsSet.size() > 3)
							{
								for (int n3 = 0; n3 < multiEntsSet[3].size(); n3++)
								{
									cout << multiEntsSet[0][n0] << " " << multiEntsSet[1][n1] << " " << multiEntsSet[2][n2] << " " << multiEntsSet[3][n3] << endl;
									baseEnt.push_back(multiEntsSet[0][n0]);
									baseEnt.push_back(multiEntsSet[1][n1]);
									baseEnt.push_back(multiEntsSet[2][n2]);
									baseEnt.push_back(multiEntsSet[3][n3]);
									combination.push_back(baseEnt);
								}
							}
							else
							{
								cout << multiEntsSet[0][n0] << " " << multiEntsSet[1][n1] << " " << multiEntsSet[2][n2] << endl;
								baseEnt.push_back(multiEntsSet[0][n0]);
								baseEnt.push_back(multiEntsSet[1][n1]);
								baseEnt.push_back(multiEntsSet[2][n2]);
								combination.push_back(baseEnt);
							}
						}
					}
					else
					{
						cout << multiEntsSet[0][n0] << " " << multiEntsSet[1][n1] << endl;
						baseEnt.push_back(multiEntsSet[0][n0]);
						baseEnt.push_back(multiEntsSet[1][n1]);
						combination.push_back(baseEnt);
					}
				}
			}
			else
			{
				cout << multiEntsSet[0][n0] << endl;
				baseEnt.push_back(multiEntsSet[0][n0]);
				combination.push_back(baseEnt);
			}
		}
		cout << "all the multiple combine end------------\n";
	}
	else
	{
		combination.push_back(singleEnts);
	}
	
// 	for (int i = 0; i < combination.size(); i++)
// 	{
// 		for (int j = 0; j < combination[i].size(); j++)
// 		{
// 			cout << combination[i][j] << " ";
// 		}cout << endl;
// 	}

	for (int i = 0; i < combination.size(); i++)
	{
		vector<Ent> ents;
		for (int j = 0; j < combination[i].size(); j++)
		{
			ents.push_back(validEnts[combination[i][j]]);
		}
		res.push_back(ents);
	}
	
// 	for (int i = 0; i < res.size(); i++)
// 	{
// 		for (int j = 0; j < res[i].size(); j++)
// 		{
// 			cout << res[i][j].dir << " ";
// 		}cout << endl;
// 	}
	
	return res;
}





















#endif











