#ifndef WORLDSTATEFEATURE_H
#define WORLDSTATEFEATURE_H

#include "stdio.h"
#include "stdlib.h"
#include <vector>
#include <cmath>
#include <fstream>

#include "Header.h"

Ent Robot::GetRobotEntity(float x, float y, float theta)
{
	Ent res;
	res.vec = GenerateRobotVector(x, y);
	res.dir = theta;
	res.name = "robot";;
	
	return res;
}

Ent Robot::GetGoEntity(int frid)
{
	Fur f = m_furLib[frid];
	string name = f.name;
	vector<float> posedim = f.posedim;
	Ent res;

	cout << posedim[0] << " " << posedim[1] << " " << posedim[2] << " " << posedim[3] << "\n";
	cout << posedim[2] << endl;
	cout << name << endl;

	
	res.vec = GenerateEntityVector(posedim[0], posedim[1], posedim[3], posedim[4]);
	res.dir = posedim[2] * PI / 180;
	res.name = name;

	return res;
}

Ent Robot::GetWallEntity(string targetRoom)
{
	Ent res;
	
	float angleSet[] = {3/2*PI, 0, PI/2, PI, PI/2, PI, 3/2*PI, 0};
	float wallPara[8][4] = {	{2.4, 8, 9.3, 0.2},
					{-2.3, 4.67, 0.2, 6.67},
					{2.4, 1.33, 9.3, 0.2},
					{7, 4.67, 0.2, 6.67},
					{0.6, -7.9, 5.6, 0.2},
					{3.4, -4.82, 0.2, 6.6},
					{0.6, -1.3, 5.6, 0.2},
					{-2.2, -4.82, 0.2, 6.6	}};
	
	//if (targetRoom.compare("bedroom") == 0)
	if (m_posRobot.GetY() > 1.3)
	{
		vector<float> walls;
		for (int i = 0; i < 4; i++)
		{
			vector<float> wall = GenerateEntityVector(wallPara[i][0], wallPara[i][1], wallPara[i][2], wallPara[i][3]);
			walls.insert( walls.end(), wall.begin(), wall.end() );
		}
		res.vec = walls;
		res.dir = -1;
	}
	//else if (targetRoom.compare("livingroom") == 0)
	else if (m_posRobot.GetY() < -1.3)
	{
		vector<float> walls;
		for (int i = 4; i < 8; i++)
		{
			vector<float> wall = GenerateEntityVector(wallPara[i][0], wallPara[i][1], wallPara[i][2], wallPara[i][3]);
			walls.insert( walls.end(), wall.begin(), wall.end() );
		}
		res.vec = walls;
		res.dir = -1;
	}
	res.name = "wall";

	return res; 
}

Ent Robot::GetRoomEntity(string roomName)
{
	Ent res;
	
	float angleSet[] = {3/2*PI, 0, PI/2, PI, PI/2, PI, 3/2*PI, 0};
	float wallPara[8][4] = {	{2.4, 8, 9.3, 0.2},
					{-2.3, 4.67, 0.2, 6.67},
					{2.4, 1.33, 9.3, 0.2},
					{7, 4.67, 0.2, 6.67},
					{0.6, -7.9, 5.6, 0.2},
					{3.4, -4.82, 0.2, 6.6},
					{0.6, -1.3, 5.6, 0.2},
					{-2.2, -4.82, 0.2, 6.6	}};
	
	if (roomName.compare("bedroom") == 0)
	{
		vector<float> walls;
		for (int i = 0; i < 4; i++)
		{
			vector<float> wall = GenerateEntityVector(wallPara[i][0], wallPara[i][1], wallPara[i][2], wallPara[i][3]);
			walls.insert( walls.end(), wall.begin(), wall.end() );
		}
		res.vec = walls;
		res.dir = 3/2*PI;
		res.name = "bedroom";
	}
	else if (roomName.compare("livingroom") == 0)
	{
		vector<float> walls;
		for (int i = 4; i < 8; i++)
		{
			vector<float> wall = GenerateEntityVector(wallPara[i][0], wallPara[i][1], wallPara[i][2], wallPara[i][3]);
			walls.insert( walls.end(), wall.begin(), wall.end() );
		}
		res.vec = walls;
		res.dir = 1/2*PI;
		res.name = "livingroom";
	}

	return res; 
}

SpR Robot::GetSpatialRelationB2A(Ent A, Ent B)
{
	SpR res;
	
	res.nameA = A.name;
	res.nameB = B.name;

	vector<float> dirhg = HistogramOfForcesWithOrientation(A.vec, B.vec, A.dir);
	vector<float> disthg = HistogramOfDistance(A.vec, B.vec);

// 	res.outdirw = OutsideDirectionWeights(dirhg);
// 	res.indirw = InsideDirectionWeights(dirhg);
	
	vector<vector<float> > outindirw = DirectionWeights(dirhg);
	res.dirhg = dirhg;
	res.disthg = disthg;
	res.outdirw = outindirw[0];
	res.indirw = outindirw[1];
	res.distw = DistanceWeights(disthg);

	return res;
}

vector<vector<float> > Robot::DirectionWeights(vector<float> hof)
{
	vector<vector<float> > res;
	vector<float> outh(hof.begin(), hof.begin()+180);
	vector<float> inh(hof.begin()+180, hof.end());
	//the summation of h. (to be normalized)
	float D = 15;

	vector<float> outdir(4, 0); //front, left, back, right
	for (int i = 0; i < 90; i++)
	{
		outdir[0] += outh[i] * Gaussian(45, D, i);
	}
	for (int i = 45; i < 135; i++)
	{
		outdir[1] += outh[i] * Gaussian(90, D, i);
	}
	for (int i = 90; i < 180; i++)
	{
		outdir[2] += outh[i] * Gaussian(135, D, i);
	}
	for (int i = 135; i < 180; i++)
	{
		outdir[3] += outh[i] * Gaussian(180, D, i);
	}
	for (int i = 0; i < 45; i++)
	{
		outdir[3] += outh[i] * Gaussian(0, D, i);
	}
	
	vector<float> indir(4, 0); //front, left, back, right
	for (int i = 0; i < 90; i++)
	{
		indir[0] += inh[i] * Gaussian(45, D, i);
	}
	for (int i = 45; i < 135; i++)
	{
		indir[1] += inh[i] * Gaussian(90, D, i);
	}
	for (int i = 90; i < 180; i++)
	{
		indir[2] += inh[i] * Gaussian(135, D, i);
	}
	for (int i = 135; i < 180; i++)
	{
		indir[3] += inh[i] * Gaussian(180, D, i);
	}
	for (int i = 0; i < 45; i++)
	{
		indir[3] += inh[i] * Gaussian(0, D, i);
	}
	
	float sumdir = 0;
	for (int i = 0; i < 4; i++)
	{
		sumdir += outdir[i] + indir[i];
	}
	for (int i = 0; i < 4; i++)
	{
		outdir[i] /= sumdir;
		indir[i] /= sumdir;
	}
	
	res.push_back(outdir);
	res.push_back(indir);

	return res;
}

vector<float> Robot::DistanceWeights(vector<float> hd)
{
 	vector<float> res;
	vector<float> h = hd;
	//the summation of h. (to be normalized)
	float hsum = 0;
	for (int i = 0; i < h.size(); i++)
	{
		hsum += h[i];
	}
	for (int i = 0; i < h.size(); i++)
	{
		h[i] /= hsum;
		//cout << i << ": " << h[i] << endl;
	}

	vector<float> distance(4, 0); //front, left, back, right
	
	distance[0] = h[0];
	
	for (int i = 0; i < h.size(); i++)
	{
		distance[1] = distance[1] + h[i] * Gaussian(10, 5, i);
		distance[2] = distance[2] + h[i] * Gaussian(25, 10, i);
		distance[3] = distance[3] + h[i] * Gaussian(60, 20, i);
	}

	float distancesum = 0;
	for (int i = 1; i < distance.size(); i++)
	{
		distancesum += distance[i];
	}
	//cout << "distancesum " << distancesum << endl;
	if (distancesum > 0)
	{
		for (int i = 1; i < distance.size(); i++)
		{
			distance[i] = distance[i] / distancesum * (1 - h[0]);
		}
	}
	else
	{
		for (int i = 1; i < distance.size(); i++)
		{
			distance[i] = 0;
		}
	}

	res = distance;
	return res;

}

float Robot::GetResponseOfADetector(SpR sp, Dct dct)
{
	float res = 0;
	
	string name = dct.nameA + "_" + dct.nameB;
	if (name.compare("rotation_rotation") == 0)
	{
		float rotsp = sp.outdirw[0];
		float rotdct = dct.outdirw[0];
		float scale = dct.indirw[0];
		
		float d = rotsp - rotdct;
		while (d > PI)	{	d -= PI*2;	}
		while (d < -PI)	{	d += PI*2;	}
		d = abs(d);
		if (d <= scale)
		{
			res = 1;
		}
		else
		{
			float g = (PI - d) / (PI - scale);
			res = Gaussian(1, 0.25, g);
		}
	}
	else
	{
		float outdirresp = 0;
		float indirresp = 0;
		float distresp = 0;
		
		for (int i = 0; i < dct.outdirw.size(); i++)
		{
			outdirresp += sp.outdirw[i] * dct.outdirw[i];
			//cout << sp.outdirw[i] << " " << dct.outdirw[i] << endl;
		}
		
		for (int i = 0; i < dct.indirw.size(); i++)
		{
			indirresp += sp.indirw[i] * dct.indirw[i];
			//cout << sp.indirw[i] << " " << dct.indirw[i] << endl;
		}
		
		for (int i = 0; i < dct.distw.size(); i++)
		{
			distresp += sp.distw[i] * dct.distw[i];
			//cout << sp.distw[i] << " " << dct.distw[i] << endl;
		}
		
		//cout << "--" << outdirresp + indirresp << " " << distresp << "--\n";
		
		res = ((outdirresp + indirresp) + distresp) / 2;
		//res = (outdirresp + indirresp) * distresp;
	}
	
	return res;
}

vector<Ent> Robot::GetVisiableEntities()
{
	vector<Ent> res;
	
	//OR
	Ent OR = GetRobotEntity(m_originalRobotPose[0], m_originalRobotPose[1], m_originalRobotPose[2]);
	OR.name = "OR";
	res.push_back(OR);

	//CR
	Ent CR = GetRobotEntity(m_posRobot.GetX(), m_posRobot.GetY(), m_theta);
	CR.name = "CR";
	res.push_back(CR);

	//Wall
	res.push_back(GetWallEntity(m_targetRoom));

	//room
	res.push_back(GetRoomEntity("bedroom"));
	res.push_back(GetRoomEntity("livingroom"));

	//Furniture
	for (int i = 0; i < m_fr.size(); i++)
	{
		if (m_fr[i] > 0)
		{
			res.push_back(GetGoEntity(m_fr[i]));
		}
	}

	//Furniture Pairs
	
	return res;
}

#endif
