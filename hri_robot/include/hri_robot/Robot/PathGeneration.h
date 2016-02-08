#ifndef ROBOTPATHGENERATION_H
#define ROBOTPATHGENERATION_H

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

map<string, vector<Dct> > Robot::LoadGroundingTypesList()
{
	map<string, vector<Dct> > res;
	
	// get the grouding types
	string rootDir = "/home/hri/hri_DATA/Targets/";
	vector<string> groundingList;
	vector<string> groundingDirList;
	
	DIR *dpdf;
	struct dirent *epdf;
	dpdf = opendir(rootDir.c_str());
	if (dpdf != NULL)
	{
		while (epdf = readdir(dpdf))
		{  
			//cout << epdf->d_name << endl;
			string dirName(epdf->d_name);
			if (dirName.size() > 5)
			{
				groundingDirList.push_back(rootDir + dirName + "/");
				groundingList.push_back(dirName);
				cout << dirName << endl;
				
				vector<string> cmd;
				int t(0);
				string c;
				do 
				{
					if (dirName[t] == '_')
					{
						cmd.push_back(c);
						c.clear();
					}
					else
					{
						c.push_back(dirName[t]);
					}
				}while(t++ < dirName.size()-1);
				cmd.push_back(c);
				cout << cmd[0] << " " << cmd[1] << " " << cmd[2] << " " << cmd[3] << " " << cmd[4] << endl;
				vector<Dct> dct = ReadDetector(cmd);
				res[dirName] = dct;
			}
		}
	}
	
	
	return res;
}

vector<string> Robot::GenerateStaticDescription(map<string, vector<Dct> > dctMap)
{
	vector<string> res;
	for(map<string, vector<Dct> >::iterator it = dctMap.begin(); it != dctMap.end(); ++it) 
	{
		if (it->first.find("move") == string::npos && it->first.find("robot") == string::npos)
		{
			cout << "-" << it->first << ":" << endl;
			float score = ScoreStateToOneGrounding(it->second);
			cout << "   The score is:    " << score << endl;
			if (score > 0.01)
			{
				res.push_back(it->first);
			}
			  
		}
	}
	return res;
}

float Robot::ScoreStateToOneGrounding(vector<Dct> decisionSpatialRelations)
{
	vector<Dct>::iterator it;
	for (it = decisionSpatialRelations.begin(); it < decisionSpatialRelations.end(); it++)
	{
		if (it->nameA.find("OR") != string::npos || it->nameB.find("OR") != string::npos
		  || it->nameA.find("rotation") != string::npos || it->nameB.find("rotation") != string::npos
		)
		{
			decisionSpatialRelations.erase(it);
		}
	}
  
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
			cout << "   entity " << requiredEntitiesNames[i] << " is not observed" << endl;
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
// 	cout << m_posRobot.GetX() << ", " << m_posRobot.GetY() << ", " << m_theta << endl;
	
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
// 			cout << A.name << " " << A.vec.size()/2 << ", " << B.name << " " << B.vec.size()/2 << endl;
			spAB = GetSpatialRelationB2A(A, B);			
			
		}
		
		float dctResponse = GetResponseOfADetector(spAB, dct);
		cout << "   " <<  dct.nameA << "_" << dct.nameB << ", " << dctResponse << endl;

		if (dctResponse < respmin)
		{
			respmin = dctResponse;
		}
	}
	
	return respmin;
}

#endif