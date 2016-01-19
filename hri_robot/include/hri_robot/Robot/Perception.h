#ifndef PERCEPTION_H
#define PERCEPTION_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include "hri_perception/Env.h"

#include "../Universal/Header.h"

#include "Header.h"

void Robot::EnvCallback(const hri_perception::Env::ConstPtr& msg)
{
	vector<Ent> tempEntList;
	m_tempSEList.clear();
	
	if (msg->name0.size() > 1)
	{
		Ent ent;
		ent.name = msg->name0;
		ent.dir = msg->dir0;
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
			m_SEList.push_back(tempEntList[i]);
		}
	
	}
	else
	{
		for (int i = 0; i < tempEntList.size(); i++)
		{
			float m = 0;
			for (int j = 0; j < m_SEList.size(); j++)
			{
				float sim = CompareTwoGOs(tempEntList[i], m_SEList[j]);
				if (sim > m)
				{
					m = sim;
				}
			}
			if (m < 0.5 )
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

float Robot::CompareTwoGOs(Ent go1, Ent go2)
{
	float res = 0;
	float dist = sqrt((go1.x - go2.x)*(go1.x - go2.x) + (go1.y - go2.y)*(go1.y - go2.y));
	
	if (go1.name.compare(go2.name) != 0 && dist < 0.4)
	{
		res = 1;
	}
	else
	{
		float map[200][200] = {};
		int tc = 0;
		int tt = 0;
		for (int i = 0; i < go1.vec.size()/2; i++)
		{
			int x = (int)(go1.vec[2*i] / (GLMAP_RESOLUTION*2) + 100);
			int y = (int)(go1.vec[2*i+1] / (GLMAP_RESOLUTION*2) + 100);
			if (x < 200 && x > 0 && y < 200 && y > 0 && map[y][x] == 0)
			{
				map[y][x] = 1;
				tt++;
			}
		}
		for (int j = 0; j < go2.vec.size()/2; j++)
		{
			int x = (int)(go2.vec[2*j] / (GLMAP_RESOLUTION*2) + 100);
			int y = (int)(go2.vec[2*j+1] / (GLMAP_RESOLUTION*2) + 100);
			if (x < 200 && x > 0 && y < 200 && y > 0 && map[y][x] == 0)
			{
				map[y][x] += 2;
				tt++;
			}
			if (x < 200 && x > 0 && y < 200 && y > 0 && map[y][x] == 1)
			{
				map[y][x] += 2;
				tc++;
			}
		}
		float t = tc / tt;
		res = t;
		
		if (dist < 0.2)
		{
			res = 1;
		}
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

#endif