#ifndef ROBOTPATHGENERATION_H
#define ROBOTPATHGENERATION_H

#include "stdio.h"
#include "stdlib.h"
#include <cstdlib>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <dirent.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "Header.h"
using namespace boost::numeric::ublas;

map<string, std::vector<Dct> > Robot::LoadGroundingTypesList(string fileName)
{
	map<string, std::vector<Dct> > res;
	
	// get the grouding types
	string rootDir = fileName; 
	std::vector<string> groundingList;
	std::vector<string> groundingDirList;
	
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
				groundingDirList.push_back(rootDir + dirName + "/");
				groundingList.push_back(dirName);
				std::cout << dirName << endl;
				
				std::vector<string> cmd;
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
				std::cout << cmd[0] << " " << cmd[1] << " " << cmd[2] << " " << cmd[3] << " " << cmd[4] << endl;
				std::vector<Dct> dct = ReadDetector(cmd);
				res[dirName] = dct;
			}
		}
	}
	
	
	return res;
}

std::vector<string> Robot::GenerateDynamicDescription(float addresseeDir, std::vector<VecPosition> pathPoints, map<string, std::vector<Dct> > dctMap)
{
  	std::vector<string> res;
	// get the target object grouding

	return res;
}

std::vector<string> Robot::GenerateStaticDescription(map<string, std::vector<Dct> > dctMap)
{
  	std::vector<string> res;
	// get the target object grouding
	res.push_back("non_obj_non_non_non");
  
	// get the room and RDT grounding
	for(map<string, std::vector<Dct> >::iterator it = dctMap.begin(); it != dctMap.end(); ++it) 
	{
		if (it->first.find("move") == string::npos && it->first.find("robot") == string::npos)
		{
			std::cout << "-" << it->first << ":" << endl;
            std::vector<float> CRPose;
            CRPose.push_back(m_posRobot.GetX());
            CRPose.push_back(m_posRobot.GetY());
            CRPose.push_back(m_theta);
            std::vector<float> ORPose;
            ORPose.push_back(0);
            ORPose.push_back(0);
            ORPose.push_back(0);
			float score = ScoreStateToOneGrounding(CRPose, m_originalRobotPose, it->second, true);
			std::cout << "   The score is:    " << score << endl;
			if (score > 0.25)
			{
				res.push_back(it->first);
			}
			  
		}
	}
	return res;
}

std::vector<string> Robot::AdjustGroundingsFormatToLGServer(std::vector<string> groundings)
{
	std::vector<string> res;
	char findIt = '_';
	
	for (int n = 0; n < groundings.size(); n++)
	{
		string sample = groundings[n];
		//std::cout << sample << endl;
		std::vector<int> dc;
		for(int i = 0; i < sample.size(); i++)
		    if(sample[i] == findIt)
			dc.push_back(i);
		 
		string roomstr = sample.substr(0,dc[0]);
		string objstr = sample.substr(dc[0]+1,dc[1]-dc[0]-1);
		string refdirstr = sample.substr(dc[1]+1, dc[3]-dc[1]-1);
		string tarstr = sample.substr(dc[3]+1);
		
		if (roomstr.compare("non") != 0)
		{
			//std::cout << roomstr << endl;
			res.push_back( roomstr );
			continue;
		}
		else if (objstr.compare("non") != 0)
		{
			//std::cout << objstr << endl;
			res.push_back( objstr );
			continue;		    
		}
		else
		{
			//std::cout << refdirstr << endl;
			//std::cout << tarstr << endl;
			if (tarstr.compare("non") != 0)
			{
				res.push_back(refdirstr + "_non");
				res.push_back("non_non_" + tarstr);
			}
			else
			{
				res.push_back(refdirstr + "_" + tarstr);
			}
		}
	}
	
	return res;
}

float Robot::ScoreStateToOneGrounding(std::vector<float> CRPose, std::vector<float> ORPose, std::vector<Dct> decisionSpatialRelations, bool isStatic)
{
    std::vector<Ent> entities = m_entities;

    if (isStatic)
    {
        std::vector<Dct>::iterator it;
        for (it = decisionSpatialRelations.begin(); it < decisionSpatialRelations.end(); it++)
        {
            //std::cout << it->nameA << " ----- " << it->nameB << endl;
            if (it->nameA.find("OR") != string::npos || it->nameB.find("OR") != string::npos
            || it->nameA.find("rotation") != string::npos || it->nameB.find("rotation") != string::npos
            )
            {
                //std::cout << "remove:" << it->nameA << " ----- " << it->nameB << endl;
                decisionSpatialRelations.erase(it);
                it--;
            }
        }
    }
  
	float res = 1;
	float res2 = 0;
	//decide if there is enough entities to make decision
	std::vector<string> requiredEntitiesNames;
	for (int i = 0; i < decisionSpatialRelations.size(); i++)
	{
		string nameA = decisionSpatialRelations[i].nameA;
		string nameB = decisionSpatialRelations[i].nameB;
		requiredEntitiesNames.push_back(nameA);
		requiredEntitiesNames.push_back(nameB);
	}  
	  
	std::vector<string> myEntitiesNames;
	myEntitiesNames.push_back("rotation");
	for (int i = 0; i < entities.size(); i++)
	{
// 		std::cout << "I have " << m_entities[i].name << endl;
		myEntitiesNames.push_back(entities[i].name);
	}
	
	std::vector<int> match(requiredEntitiesNames.size(), 0);
	for (int i = 0; i < requiredEntitiesNames.size(); i++)
	{
// 		std::cout << "I need " << requiredEntitiesNames[i] << endl;
		for (int j = 0; j < myEntitiesNames.size(); j++)
		{
			if (requiredEntitiesNames[i].compare(myEntitiesNames[j]) == 0)
			{
				match[i] = 1;
				break;
			}
		}
	}
	
	for (int i = 0; i < match.size(); i++)
	{
		if (match[i] == 0)
		{
			std::cout << "   entity " << requiredEntitiesNames[i] << " is not observed" << endl;
			res = 0;
			return res;
		}
	}
							  
	SpR rot;
	rot.nameA = "rotation";
	rot.nameB = "rotation";
	float rotationf = CRPose[2] - ORPose[2];
	while (rotationf > PI)	{	rotationf -= 2*PI;	}
	while (rotationf < -PI)	{	rotationf += 2*PI;	}
	std::vector<float> rotoutdirw(1, rotationf);
	std::vector<float> rotindirw(1, 0);
	rot.outdirw = rotoutdirw;
	rot.indirw = rotindirw;
	
	for (int i = 0; i < entities.size(); i++)
	{
		if (entities[i].name.compare("CR") == 0)
		{
			entities[i] = GetRobotEntity( CRPose[0], CRPose[1], CRPose[2] );
			entities[i].name = "CR";
		}
        
 		if (entities[i].name.compare("OR") == 0)
		{
			entities[i] = GetRobotEntity( ORPose[0], ORPose[1], ORPose[2] );
			entities[i].name = "OR";
		}       
	}
// 	std::cout << m_posRobot.GetX() << ", " << m_posRobot.GetY() << ", " << m_theta << endl;
	
	float resp_min = 1;
	float resp_mul = 1;
	float resp_mean = 0;
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
// 			std::cout << A.name << " " << A.vec.size()/2 << ", " << B.name << " " << B.vec.size()/2 << endl;
			spAB = GetSpatialRelationB2A(A, B);			
			
		}
		
		float dctResponse = GetResponseOfADetector(spAB, dct);
		std::cout << "   " <<  dct.nameA << "_" << dct.nameB << ", " << dctResponse << endl;

		if (dctResponse < resp_min)
		{
			resp_min = dctResponse;
		}
        resp_mul *= dctResponse;
        resp_mean += dctResponse;
	}
	resp_mean /= decisionSpatialRelations.size();
	
	res = resp_min;
	return res;
}



int Robot::SaveEntitiesInformationToTXT(string fileName, std::vector<Ent> entities)
{
	std::ofstream out(fileName.c_str());
	for (int i = 0; i < entities.size(); i++)
	{
		out << entities[i].name << ": " << entities[i].x << " " << entities[i].y << " " << entities[i].dir << " "
		    << entities[i].confidence << " " << entities[i].vec.size()/2 << endl;
		for (int j = 0; j < entities[i].vec.size()/2; j++)
		{
			out << entities[i].vec[2*j] << " " << entities[i].vec[2*j+1] << endl;
		}
	}
	
	return entities.size();
}

std::vector<Ent> Robot::ReadEntitiesInformationFromTXT(string fileName)
{
	std::vector<Ent> res;
	std::ifstream in(fileName.c_str());
	std::vector<std::string> lines;
	std::string line;
	while(getline(in, line))
		lines.push_back(line);
	cout << lines.size() << endl;
	std::vector<int> entityHeads;
	for (int i = 0; i < lines.size(); i++)
	{
		if (lines[i].find(":") != std::string::npos)
		{
			entityHeads.push_back(i);
		}
	}
	entityHeads.push_back(lines.size());
	for (int i = 0; i < entityHeads.size()-1; i++)
	{
		Ent ent;
		int entityhead = entityHeads[i];
		int entitytail = entityHeads[i+1];
		string head = lines[entityhead];
		int vecSize = entitytail - entityhead - 1;
		std::stringstream ss(head);
		ss >> ent.name >> ent.x >> ent.y >> ent.dir >> ent.confidence >> vecSize;
		ent.name.erase(ent.name.size()-1);
		float x,y = 0;
		for (int i = 1; i <= vecSize; i++)
		{	
			std::stringstream ss(lines[entityhead+i]);
			ss >> x >> y;
			ent.vec.push_back(x);
			ent.vec.push_back(y);
		}
		std::cout << ent.name << "- "
		    << ent.x << " " << ent.y << " " << ent.dir << " "
		    << ent.confidence << " " << ent.vec.size()/2 << endl;
		
		res.push_back(ent);
	}
	return res;
}

void Robot::AnalyseEntityRelation()
{   
	for (int i = 0; i < m_entities.size(); i++)
	{
		std::cout << " -" << m_entities[i].name << ": " 
			  << m_entities[i].vec.size()/2 << ", " 
			  << m_entities[i].x << ", " << m_entities[i].y << ",   " 
			  << m_entities[i].dir << endl;
			  
			  m_entities[i].id = i;
 	}
     
	int L = m_entities.size();
	matrix<double> m (L, L);
	int t = 0;

	std::cout << m << std::endl;
}



#endif