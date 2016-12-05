#ifndef PERCEPTION_H
#define PERCEPTION_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include "hri_perception/Env.h"

#include "../Universal/Header.h"

#include "Header.h"

void Robot::CallForPercepstionService()
{
	hri_perception::Perception srv;
	srv.request.fetch = true;
	
	if (m_perceptionClient.call(srv))
	{	
		hri_perception::Env msg = srv.response.env;
	  
		vector<Ent> tempEntList;
		m_tempSEList.clear();
		
		if (msg.name0.size() > 1)
		{
			Ent ent;
			ent.name = msg.name0;
			ent.dir = msg.dir0;
			ent.confidence = msg.conf0;
			ent.vec.insert(ent.vec.end(), &msg.vec0[0], &msg.vec0[msg.vec0.size()]);
			for (int i = 0; i < ent.vec.size()/2; i++)
			{
				ent.x += ent.vec[2*i];
				ent.y += ent.vec[2*i+1];
			}
			ent.x /= ent.vec.size()/2;
			ent.y /= ent.vec.size()/2;
			tempEntList.push_back(ent);
		}
		
		if (msg.name1.size() > 1)
		{
			Ent ent;
			ent.name = msg.name1;
			ent.dir = msg.dir1;
			ent.confidence = msg.conf1;
			ent.vec.insert(ent.vec.end(), &msg.vec1[0], &msg.vec1[msg.vec1.size()]);
			for (int i = 0; i < ent.vec.size()/2; i++)
			{
				ent.x += ent.vec[2*i];
				ent.y += ent.vec[2*i+1];
			}
			ent.x /= ent.vec.size()/2;
			ent.y /= ent.vec.size()/2;
			tempEntList.push_back(ent);
		}
		
		if (msg.name2.size() > 1)
		{
			Ent ent;
			ent.name = msg.name2;
			ent.dir = msg.dir2;
			ent.confidence = msg.conf2;
			ent.vec.insert(ent.vec.end(), &msg.vec2[0], &msg.vec2[msg.vec2.size()]);
			for (int i = 0; i < ent.vec.size()/2; i++)
			{
				ent.x += ent.vec[2*i];
				ent.y += ent.vec[2*i+1];
			}
			ent.x /= ent.vec.size()/2;
			ent.y /= ent.vec.size()/2;
			tempEntList.push_back(ent);
		}
		
		if (msg.name3.size() > 1)
		{
			Ent ent;
			ent.name = msg.name3;
			ent.dir = msg.dir3;
			ent.confidence = msg.conf3;
			ent.vec.insert(ent.vec.end(), &msg.vec3[0], &msg.vec3[msg.vec3.size()]);
			for (int i = 0; i < ent.vec.size()/2; i++)
			{
				ent.x += ent.vec[2*i];
				ent.y += ent.vec[2*i+1];
			}
			ent.x /= ent.vec.size()/2;
			ent.y /= ent.vec.size()/2;
			tempEntList.push_back(ent);
		}
		
		if (msg.name4.size() > 1)
		{
			Ent ent;
			ent.name = msg.name4;
			ent.dir = msg.dir4;
			ent.confidence = msg.conf4;
			ent.vec.insert(ent.vec.end(), &msg.vec4[0], &msg.vec4[msg.vec4.size()]);
			for (int i = 0; i < ent.vec.size()/2; i++)
			{
				ent.x += ent.vec[2*i];
				ent.y += ent.vec[2*i+1];
			}
			ent.x /= ent.vec.size()/2;
			ent.y /= ent.vec.size()/2;
			tempEntList.push_back(ent);
		}
		
		if (msg.name5.size() > 1)
		{
			Ent ent;
			ent.name = msg.name5;
			ent.dir = msg.dir5;
			ent.confidence = msg.conf5;
			ent.vec.insert(ent.vec.end(), &msg.vec5[0], &msg.vec5[msg.vec5.size()]);
			for (int i = 0; i < ent.vec.size()/2; i++)
			{
				ent.x += ent.vec[2*i];
				ent.y += ent.vec[2*i+1];
			}
			ent.x /= ent.vec.size()/2;
			ent.y /= ent.vec.size()/2;
			tempEntList.push_back(ent);
		}
		
	// 	cout << "ent num: " << tempEntList.size() << endl;
		m_tempSEList = tempEntList; 
		
		cout << "m_tempSEList" << endl;
		for (int i = 0; i < m_tempSEList.size(); i++)
		{
			  cout << m_tempSEList[i].name << " " << m_tempSEList[i].x << " " << m_tempSEList[i].y << " " << m_tempSEList[i].dir << " " << m_tempSEList[i].confidence << endl;
		}

	}
	else
	{
		ROS_DEBUG_ONCE("Failed to call service Perception");
		return;
	}
  
	m_ifGetPerception = true;
}

void Robot::EnvCallback(const hri_perception::Env::ConstPtr& msg)
{
	vector<Ent> tempEntList;
	m_tempSEList.clear();
	
	if (msg->name0.size() > 1)
	{
		Ent ent;
		ent.name = msg->name0;
		ent.dir = msg->dir0;
		ent.confidence = msg->conf0;
		ent.vec.insert(ent.vec.end(), &msg->vec0[0], &msg->vec0[msg->vec0.size()]);
		for (int i = 0; i < ent.vec.size()/2; i++)
		{
			ent.x += ent.vec[2*i];
			ent.y += ent.vec[2*i+1];
		}
		ent.x /= ent.vec.size()/2;
		ent.y /= ent.vec.size()/2;
		tempEntList.push_back(ent);
	}
	
	if (msg->name1.size() > 1)
	{
		Ent ent;
		ent.name = msg->name1;
		ent.dir = msg->dir1;
		ent.confidence = msg->conf1;
		ent.vec.insert(ent.vec.end(), &msg->vec1[0], &msg->vec1[msg->vec1.size()]);
		for (int i = 0; i < ent.vec.size()/2; i++)
		{
			ent.x += ent.vec[2*i];
			ent.y += ent.vec[2*i+1];
		}
		ent.x /= ent.vec.size()/2;
		ent.y /= ent.vec.size()/2;
		tempEntList.push_back(ent);
	}
	
	if (msg->name2.size() > 1)
	{
		Ent ent;
		ent.name = msg->name2;
		ent.dir = msg->dir2;
		ent.confidence = msg->conf2;
		ent.vec.insert(ent.vec.end(), &msg->vec2[0], &msg->vec2[msg->vec2.size()]);
		for (int i = 0; i < ent.vec.size()/2; i++)
		{
			ent.x += ent.vec[2*i];
			ent.y += ent.vec[2*i+1];
		}
		ent.x /= ent.vec.size()/2;
		ent.y /= ent.vec.size()/2;
		tempEntList.push_back(ent);
	}
	
	if (msg->name3.size() > 1)
	{
		Ent ent;
		ent.name = msg->name3;
		ent.dir = msg->dir3;
		ent.confidence = msg->conf3;
		ent.vec.insert(ent.vec.end(), &msg->vec3[0], &msg->vec3[msg->vec3.size()]);
		for (int i = 0; i < ent.vec.size()/2; i++)
		{
			ent.x += ent.vec[2*i];
			ent.y += ent.vec[2*i+1];
		}
		ent.x /= ent.vec.size()/2;
		ent.y /= ent.vec.size()/2;
		tempEntList.push_back(ent);
	}
	
	if (msg->name4.size() > 1)
	{
		Ent ent;
		ent.name = msg->name4;
		ent.dir = msg->dir4;
		ent.confidence = msg->conf4;
		ent.vec.insert(ent.vec.end(), &msg->vec4[0], &msg->vec4[msg->vec4.size()]);
		for (int i = 0; i < ent.vec.size()/2; i++)
		{
			ent.x += ent.vec[2*i];
			ent.y += ent.vec[2*i+1];
		}
		ent.x /= ent.vec.size()/2;
		ent.y /= ent.vec.size()/2;
		tempEntList.push_back(ent);
	}
	
	if (msg->name5.size() > 1)
	{
		Ent ent;
		ent.name = msg->name5;
		ent.dir = msg->dir5;
		ent.confidence = msg->conf5;
		ent.vec.insert(ent.vec.end(), &msg->vec5[0], &msg->vec5[msg->vec5.size()]);
		for (int i = 0; i < ent.vec.size()/2; i++)
		{
			ent.x += ent.vec[2*i];
			ent.y += ent.vec[2*i+1];
		}
		ent.x /= ent.vec.size()/2;
		ent.y /= ent.vec.size()/2;
		tempEntList.push_back(ent);
	}
	
// 	cout << "ent num: " << tempEntList.size() << endl;
	m_tempSEList = tempEntList;
	m_ifGetPerception = true;
}

int Robot::Perception()
{
	int res = 0;
	vector<Ent> tempGlobalSEList;
	for (int i = 0; i < m_tempSEList.size(); i++)
	{
		tempGlobalSEList.push_back(GenerateGlobalEnt(m_tempSEList[i]));
	}
	UpdateSEMap(tempGlobalSEList);
	return res;
}

int Robot::UpdateSEMap(vector<Ent> tempEntList)
{
	int res = 0;
	
	if (m_SEList.size() == 0)
	{
		for (int i = 0; i < tempEntList.size(); i++)
		{
			if (tempEntList[i].name.compare("unknown") != 0 && tempEntList[i].confidence > 0.6)
			{
				m_SEList.push_back(tempEntList[i]);
			}
		}
	}
	else
	{
		for (int i = 0; i < tempEntList.size(); i++)
		{
			float m = 0;
			float besti = 0;
			int r = 0;
			int r2 = 0;
			for (int j = 0; j < m_SEList.size(); j++)
			{
				r =  CompareTwoGOs(tempEntList[i], m_SEList[j]);
				if (r == 1)
				{
				    m_SEList[j] = tempEntList[i];
				    break;
				}  
				if (r == 2)
				{
				    r2++;
				}
			}
			if (r2 == m_SEList.size())
			{
				m_SEList.push_back(tempEntList[i]);  
			}
		}
	}
	
	cout << "current SE list: " << m_SEList.size() << endl;
 	for (int i = 0; i < m_SEList.size(); i++)
 	{
 		cout << " --" << m_SEList[i].name << " " << m_SEList[i].x << " " << m_SEList[i].y << " " << m_SEList[i].dir << endl;
 	}
	
	res = m_SEList.size();
	
	return res;
}

int Robot::CompareTwoGOs(Ent go1, Ent go2)
{
	int res = 0;
	float dist = sqrt((go1.x - go2.x)*(go1.x - go2.x) + (go1.y - go2.y)*(go1.y - go2.y));
	
	//calculate the probability to decide if the go2 should be replaced by go1.
	float map1and2[200][200] = {};
	float map1or2[200][200] = {};
	float map1[200][200] = {};
	float map2[200][200] = {};
	
	for (int i = 0; i < go1.vec.size()/2; i++)
	{
		int x = (int)(go1.vec[2*i] / (GLMAP_RESOLUTION*2) + 100);
		int y = (int)(go1.vec[2*i+1] / (GLMAP_RESOLUTION*2) + 100);
		if (x < 200 && x > 0 && y < 200 && y > 0 && map1[y][x] == 0)
		{
			map1[y][x] = 1;
			map1or2[y][x] = 1;
			if (map2[y][x] == 1)
				map1and2[y][x] = 1;
		}
	}
	for (int i = 0; i < go2.vec.size()/2; i++)
	{
		int x = (int)(go2.vec[2*i] / (GLMAP_RESOLUTION*2) + 100);
		int y = (int)(go2.vec[2*i+1] / (GLMAP_RESOLUTION*2) + 100);
		if (x < 200 && x > 0 && y < 200 && y > 0 && map2[y][x] == 0)
		{
			map2[y][x] = 1;
			map1or2[y][x] = 1;
			if (map1[y][x] == 1)
				map1and2[y][x] = 1;
		}
	}
	
	float area1 = 0;
	float area2 = 0;
	float areaand = 0;
	float areaor = 0;
	for (int y = 0; y < 200; y++)
	{
		for (int x = 0; x < 200; x++)
		{
			if (map1[y][x] == 1)
				area1++;
			if (map2[y][x] == 1)
				area2++;
			if (map1and2[y][x] == 1)
				areaand++;
			if (map1or2[y][x] == 1)
				areaor++;
		}
	}
	
	if (go1.name != "unknown")
	{
		if (areaand > 0.5 * area1)
		{
			if (areaand > 0.3 * area2)
			{
				if (go1.confidence > 1.2 * go2.confidence)
				{
					res = 1;
				}
			}
		}
		
		if (areaand <= 0.1 * area1 && areaand < 0.1 * area2)
		{
			res = 2;
		}
		cout << area1 << " " << area2 << " " << areaand << " " << areaor << " " << res << endl;
	} 
	
	return res;
}

Ent Robot::GenerateGlobalEnt(Ent le)
{
	Ent ge;
	
	ge.name = le.name;
	//ge.confidence = g.m_confidence;
	vector<float> xyth = LocalToGlobal(le.x, le.y, le.dir);
	ge.x = xyth[0];
	ge.y = xyth[1];
	if (le.dir > 0)
	{
		ge.dir = xyth[2];
	}
	else
	{
		ge.dir = -1;
	}
	
	if (ge.name.compare("bed") == 0)
	{
		ge.dir = PI / 2;
	}
	
	vector<float> vec;
	for (int i = 0; i < le.vec.size()/2; i++)
	{
		vector<float> gxyt = LocalToGlobal(le.vec[2*i], le.vec[2*i+1], 0);
		vec.push_back(gxyt[0]);
		vec.push_back(gxyt[1]);
	}
	ge.vec = vec;
	return ge;
}

int Robot::DrawOccupancyGrid()
{
	float originX = -10;
	float originY = -10;
	float resolution = 0.1;
	Ent ent;
	m_imgGrid = cv::Scalar(0, 0, 0);
	
	map<string, vector<unsigned char> > colorTuple;
	unsigned char bgrTable[3] = {255, 0, 0};
	unsigned char bgrChair[3] = {0, 255, 0};
	unsigned char bgrCouch[3] = {0, 0, 255};
	unsigned char bgrBed[3] = {255, 255, 0};
	unsigned char bgrRoom[3] = {150, 150, 255};
	colorTuple["table"] = *(new vector<unsigned char>(bgrTable, bgrTable + sizeof(bgrTable) / sizeof(unsigned char)));
	colorTuple["chair"] = *(new vector<unsigned char>(bgrChair, bgrChair + sizeof(bgrChair) / sizeof(unsigned char)));
	colorTuple["couch"] = *(new vector<unsigned char>(bgrCouch, bgrCouch + sizeof(bgrCouch) / sizeof(unsigned char)));
	colorTuple["bed"] = *(new vector<unsigned char>(bgrBed, bgrBed + sizeof(bgrBed) / sizeof(unsigned char)));
	colorTuple["room"] = *(new vector<unsigned char>(bgrRoom, bgrRoom + sizeof(bgrRoom) / sizeof(unsigned char)));
	
	for (int n = 0; n < m_entities.size(); n++)
	{
		if (m_entities[n].name == "CR" || m_entities[n].name == "OR")
		{
		    continue;
		}
		
		ent = m_entities[n];
		int u, v, p;
		string name = m_entities[n].name;
		for (int i = 0; i < ent.vec.size() / 2; i++)
		{
			u = (int)((ent.vec[2*i] - originX) / resolution);
			v = (int)((ent.vec[2*i+1] - originY) / resolution);
			//cout << u << ", " << v << endl;
			if (u >= 0 && u < m_imgGrid.cols && v >= 0 && v < m_imgGrid.rows)
			{			
				p = u + v * 200;
				m_imgOccupancy.data[p] = 255;
				
				if (colorTuple.count(name) == 1)
				{
					m_imgGrid.data[3*p] = (colorTuple[name])[0];
					m_imgGrid.data[3*p+1] = (colorTuple[name])[1];
					m_imgGrid.data[3*p+2] = (colorTuple[name])[2];
				}
				else
				{
					m_imgGrid.data[3*p] = 255;
					m_imgGrid.data[3*p+1] = 255;
					m_imgGrid.data[3*p+2] = 255;				
				}
			}
		}
	}
	
	int res;
	res = m_entities.size();
	return res;
}

#endif