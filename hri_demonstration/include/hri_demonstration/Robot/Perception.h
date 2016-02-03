#ifndef PERCEPTION_H
#define PERCEPTION_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>

#include "Header.h"

using namespace std;

int Robot::Perception()
{
	int res = 0;

	// detect furniture
	int N = m_laserUnitNumber;
	vector<float> objIDset;
	for (int i = 0; i < N; i++)
	{
		vector<float> laserUnit = m_laserData[i];
		int ID = ((int)(laserUnit[2]));
		
		if (ID >= 100)
		{
			float lx = laserUnit[1] * cos(laserUnit[0]);
			float ly = laserUnit[1] * sin(laserUnit[0]);
			vector<float> gxy = LocalToGlobal(lx, ly, 0);
			vector<int> p = CoordinateToPixel( gxy[0], gxy[1]);
			Vec3b intensity;
			intensity.val[0] = 200;
			intensity.val[1] = 200;
			intensity.val[2] = 200;
			m_imgmap.at<Vec3b>(p[1], p[0]) = intensity;
			objIDset.push_back(ID);
		}
	}

	sort(objIDset.begin(), objIDset.end());
	objIDset.erase(unique(objIDset.begin(), objIDset.end()), objIDset.end()); 
	
/*
	if (objIDset.size() > 0)
	{
		for (int i = 0; i < objIDset.size(); i++)
		{
			cout << objIDset[i] << " ";
		}
		cout << endl;		
	}
*/

	for (int i = 0; i < objIDset.size(); i++)
 	{
		m_fr.push_back(objIDset[i]);
	}
	sort(m_fr.begin(), m_fr.end()); // lala: 1, 3, 99, 99
	m_fr.erase(unique(m_fr.begin(), m_fr.end()), m_fr.end()); 
	res = m_fr.size();
	
	return res;
}

int Robot::UpdateOccupyMap()
{
	return 0;
}

#endif
