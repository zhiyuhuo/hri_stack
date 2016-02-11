#ifndef HOF_H
#define HOF_H

#include "stdio.h"
#include "stdlib.h"

#include <vector>
#include <iostream>
#include <math.h>
#include <time.h>

#include "Header.h"

using namespace std;

vector<float> HistogramOfDistance(vector<float> B, vector<float> A)
{
	vector<float> res(100, 0);
	for (int i = 0; i < B.size()/2; i++)
	{
		float maxb = 100;
		for (int j = 0; j < A.size()/2; j++)
		{
			float dist = sqrt( pow(A[2*j]-B[2*i], 2) + pow(A[2*j+1]-B[2*i+1], 2) );
			if (dist < maxb)
			{
				maxb = dist;
			}
		}
		
		int k = maxb / 0.1;
		if (k < res.size())
		{
			res[k]++;
		}
	}
	
		
	for (int i = 0; i < res.size(); i++)
	{
		res[i] /= B.size()/2;
	}
	
	return res;
}

vector<float> HistogramOfForces(vector<float> B, vector<float> A) //Hof A to B.
{
	float h[180] = {};
	int LA = A.size() / 2;
	int LB = B.size() / 2;

	for (int i = 0; i < LB; i++)
	{
		for (int j = 0; j < LA; j++)
		{
			float dx = float(A[2*j] - B[2*i]);
			float dy = float(A[2*j+1] - B[2*i+1]);
			float tf = atan2(dy,dx);
			int ti = int(tf * 180 / (2*PI));
			if (ti < 0)
			{
				ti += 180;
			}

			if (ti >= 0 && ti < 180)
			{
				h[ti]++;
			}
		}
	}
	vector<float> res(h, h + 180);
// 	for (int i = 0; i < res.size(); i++)
// 	{
// 		cout << i << ": " << res[i] << endl;
// 	}
// 	cout << A.size()/2 << " " << B.size()/2 << endl;
	return res;
}

vector<float> HistogramOfForces2b(vector<float> B, vector<float> A) //Hof A to B.
{
	float h[360] = {};
	int LA = A.size() / 2;
	int LB = B.size() / 2;
	
	float xBmin = 1000;
	float xBmax = -1000;
	float yBmin = 1000;
	float yBmax = -1000;
	
	for (int i = 0; i < LB; i++)
	{
		if (B[2*i] < xBmin)
		{
			xBmin = B[2*i];
		}
		if (B[2*i] > xBmax)
		{
			xBmax = B[2*i];
		}
		if (B[2*i+1] < yBmin)
		{
			yBmin = B[2*i+1];
		}
		if (B[2*i+1] > yBmax)
		{
			yBmax = B[2*i+1];
		}
	}

	for (int i = 0; i < LB; i++)
	{
		for (int j = 0; j < LA; j++)
		{
			if (A[2*j] == B[2*i] && A[2*j+1] == B[2*i+1])
			{
				for (int k = 180; k < 360; k++)
				{
					h[k] += 1 / 0.05;
				}
			}
			else
			{
				float dx = float(A[2*j] - B[2*i]);
				float dy = float(A[2*j+1] - B[2*i+1]);
				float d = sqrt( dx*dx + dy*dy );
				if (d < 0.05)
				{
					d = 0.025;
				}
				float tf = atan2(dy,dx);
				if (tf < 0)
				{
					tf += 2 * PI;
				}
				int ti = (int)(tf * 180 / PI / 2);

				if (ti >= 0 && ti <= 179)
				{
					if (A[2*j] >= xBmin && A[2*j] <= xBmax && A[2*j+1] >= yBmin && A[2*j+1] <= yBmax)
					{
						h[ti+180] += 1;
					}
					else
					{
						h[ti] += 1;
					}				
				}	
			}
		}
	}
	
	float sumh = 0;
	for (int i = 0; i < 360; i++)
	{
		sumh += h[i];
	}
	for (int i = 0; i < 360; i++)
	{
		h[i] /= sumh;
	}
	
	vector<float> res(h, h + 360);
	return res;
}

vector<float> HistogramOfForces2(vector<float> A, vector<float> B) //Hof B to A.
{
	float h[360] = {};
	int LA = A.size() / 2;
	int LB = B.size() / 2;
	
	float xAmin = 1000;
	float xAmax = -1000;
	float yAmin = 1000;
	float yAmax = -1000;
	
	for (int i = 0; i < LA; i++)
	{
		if (A[2*i] < xAmin)
		{
			xAmin = A[2*i];
		}
		if (A[2*i] > xAmax)
		{
			xAmax = A[2*i];
		}
		if (A[2*i+1] < yAmin)
		{
			yAmin = A[2*i+1];
		}
		if (A[2*i+1] > yAmax)
		{
			yAmax = A[2*i+1];
		}
	}

	float Bout = 0;
	for (int i = 0; i < LB; i++)
	{
	  	if (B[2*i] >= xAmin - 0.01 && B[2*i] <= xAmax + 0.01 && B[2*i+1] >= yAmin - 0.01 && B[2*i+1] <= yAmax + 0.01)
		{
			Bout ++;
		}
// 		cout << B[2*i] << ", " << B[2*i+1] << endl;;
		for (int j = 0; j < LA; j++)
		{
			if (abs(A[2*j] -  B[2*i]) < 0.01 && abs(A[2*j+1] - B[2*i+1]) < 0.01)
			{
				for (int k = 180; k < 360; k++)
				{
					h[k] += 1 / (0.05 * 0.05);
				}
			}
			else
			{
				float dx = float(B[2*i] - A[2*j]);
				float dy = float(B[2*i+1] - A[2*j+1]);
				float d = sqrt( dx*dx + dy*dy );
				if (d < 0.05)
				{
					d = 0.05;
				}
				float tf = atan2(dy,dx);
				if (tf < 0)
				{
					tf += 2 * PI;
				}
				if (tf >= 2*PI)
				{
					tf -= 2 * PI;
				}
				int ti = (int)(tf * 180 / PI / 2);

				if (ti >= 0 && ti <= 179)
				{
					if (B[2*i] >= xAmin - 0.01 && B[2*i] <= xAmax + 0.01 && B[2*i+1] >= yAmin - 0.01 && B[2*i+1] <= yAmax + 0.01)
					{
						h[ti+180] += 1 / (d*d);
					}
					else
					{
// 						cout << B[2*i] << ", " << B[2*i+1] << " - " << xAmin << " " << xAmax << " " << yAmin << " " << yAmax << endl;
						h[ti] += 1 / (d*d);
					}				
				}	
			}
		}
	}
	
	float sumh180 = 0;
	float sumh360 = 0;
	for (int i = 0; i < 180; i++)
	{
		sumh180 += h[i];
	}
	for (int i = 180; i < 360; i++)
	{
		sumh360 += h[i];
	}
	
	if (sumh180 > 0)
	{	
		for (int i = 0; i < 180; i++)
		{
			h[i] /= sumh180 * (LB-Bout) / LB;
		}	
	}
	if (sumh360 > 0)
	{
		for (int i = 180; i < 360; i++)
		{
			h[i] /= sumh360 * Bout / LB;
		}
	}
	
// 	for (int i = 0; i < 360; i++)
// 	{
// 		cout << i << ": " << h[i] << endl;
// 	}
	
	vector<float> res(h, h + 360);
// 	exit(1);
	return res;
}

vector<float> GenerateEntityVector(float px, float py, float dimx, float dimy)
{
	vector<float> res;

	for (float i = -dimx/2; i <= dimx/2; i += 0.1)
	{
		for (float j = -dimy/2; j <= dimy/2; j += 0.1)
		{
			res.push_back(px+i);
			res.push_back(py+j);
		}
	}

	return res;	
}

vector<float> GenerateRobotVector(float px, float py)
{
	vector<float> res = GenerateEntityVector(px, py, 0.3, 0.3);
	return res;
}

vector<float> GenerateOriginRobotVector()
{
	vector<float> res;

	float ix = 0;
	float iy = 0;

	for (float i = -10; i < 10; i += 0.05)
	{
		for (float j = -10; j < 10; j += 0.05)
		{
			float d = sqrt(pow(i - ix, 2) + pow(j - iy, 2));
			if (d <= 0.2)
			{
				res.push_back(i);
				res.push_back(j);
			}
		}
	}

	return res;	
}

vector<float> HistogramOfForcesWithOrientation(vector<float> B, vector<float> A, float Borientation)
{
	vector<float> res;

	vector<float> B2(B.size(), 0);
	float centerBX = 0;
	float centerBY = 0;
	for (int i = 0; i < B.size()/2; i++)
	{
		centerBX += B[2*i];
		centerBY += B[2*i+1];
	}
	centerBX /= B.size()/2;
	centerBY /= B.size()/2;

	for (int i = 0; i < B.size()/2; i++)
	{
		B2[2*i] = B[2*i] - centerBX;
		B2[2*i+1] = B[2*i+1] - centerBY;
	}

	vector<float> A2(A.size(), 0);
	for (int i = 0; i < A.size()/2; i++)
	{
		A2[2*i] = A[2*i] - centerBX;
		A2[2*i+1] = A[2*i+1] - centerBY;
	}

	float dth = Borientation - PI/2;
	for (int i = 0; i < A.size()/2; i++)
	{
		float x = A2[2*i];
		float y = A2[2*i+1];
		A2[2*i] = x * cos(dth) + y * sin(dth);
		A2[2*i+1] = -x * sin(dth) + y * cos(dth);
	}

	res = HistogramOfForces2(B2, A2);

	return res;
}


vector<float> HofToDirection(vector<float> hof)
{
	vector<float> res;
	//the summation of hof. (to be normalized)
	float hofsum = 0;
	for (int i = 0; i < hof.size(); i++)
	{
		hofsum += hof[i];
	}
	for (int i = 0; i < hof.size(); i++)
	{
		hof[i] /= hofsum;
	}

	float v4[4] = {0, 0, 0, 0};
	vector<float> direction(v4, v4+4); //front, left, back, right
	for (int i = 0; i < 90; i++)
	{
		direction[0] += hof[i] * Gaussian(45, 45, i);
	}
	for (int i = 45; i < 135; i++)
	{
		direction[1] += hof[i] * Gaussian(90, 45, i);
	}
	for (int i = 90; i < 180; i++)
	{
		direction[2] += hof[i] * Gaussian(135, 45, i);
	}
	for (int i = 135; i < 180; i++)
	{
		direction[3] += hof[i] * Gaussian(180, 45, i);
	}
	for (int i = 0; i < 45; i++)
	{
		direction[3] += hof[i] * Gaussian(0, 45, i);
	}

	float directionsum = 0;
	for (int i = 0; i < direction.size(); i++)
	{
		directionsum += direction[i];
	}
	for (int i = 0; i < direction.size(); i++)
	{
		direction[i] /= directionsum;
	}

	res = direction;
	return res;
}

vector<float> HofToDirectionEight(vector<float> hof)
{
	vector<float> res;
	//the summation of hof. (to be normalized)
	float hofsum = 0;
	for (int i = 0; i < hof.size(); i++)
	{
		hofsum += hof[i];
	}
	for (int i = 0; i < hof.size(); i++)
	{
		hof[i] /= hofsum;
	}

	vector<float> direction(8, 0); //front, front-left, left, left-back, back, back-right, right, right-front
	for (int i = 22; i < 68; i++) // front
	{
		direction[0] += hof[i] * Gaussian(45, 23, i);
	}
	for (int i = 45; i < 91; i++) // front-left
	{
		direction[1] += hof[i] * Gaussian(68, 23, i);
	}
	for (int i = 68; i < 114; i++) // left
	{
		direction[2] += hof[i] * Gaussian(91, 23, i);
	}
	for (int i = 90; i < 136; i++) //left-back
	{
		direction[3] += hof[i] * Gaussian(113, 23, i);
	}
	for (int i = 112; i < 158; i++) //back
	{
		direction[4] += hof[i] * Gaussian(135, 23, i);
	}
	for (int i = 135; i < 180; i++) //back-right
	{
		direction[5] += hof[i] * Gaussian(158, 23, i);
	}
	
	for (int i = 158; i < 180; i++) //right
	{
		direction[6] += hof[i] * Gaussian(180, 23, i);
	}	
	for (int i = 0; i < 23; i++) //right
	{
		direction[6] += hof[i] * Gaussian(0, 23, i);
	}
	
	for (int i = 0; i < 46; i++) //right-front
	{
		direction[7] += hof[i] * Gaussian(23, 23, i);
	}

	float directionsum = 0;
	for (int i = 0; i < direction.size(); i++)
	{
		directionsum += direction[i];
	}
	for (int i = 0; i < direction.size(); i++)
	{
		direction[i] /= directionsum;
	}

	res = direction;
	return res;
}

string DirectionToTextEight(vector<float> direction)
{
	string res;
	vector<int> indecs = RankArray(direction);
	string dirstrset[9] = {"front", "front-left", "left", "left-back", "back", "back-right", "right", "right-front", "none"};
	res = dirstrset[indecs[0]];
	
	if (direction[indecs[0]] == 0 || isnan(direction[indecs[0]]))
	{
		res = dirstrset[8];
	}
	
	float dirWeightMin = 10;
	for (int i = 0; i < 8; i++)
	{	
		if (direction[i] < dirWeightMin)
		{
			dirWeightMin = direction[i];
		}
	}
	
	if (dirWeightMin > 0)
	{
		res = dirstrset[8];
	}
	
	return res;
}

float DistanceBetweenVector(vector<float> v1, vector<float> v2)
{
	float res;
	float mind = 100;
	for (int i = 0; i < v1.size()/2; i++)
	{
		for (int j = 0; j < v2.size()/2; j++)
		{
			float x1 = v1[2*i];
			float y1 = v1[2*i+1];
			float x2 = v2[2*j];
			float y2 = v2[2*j+1];
			float d = sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
			if (d < mind)
			{
				mind = d;
			}
		}
	}
	
	res = mind;
	return res;
}

string DistanceToText(float dist)
{
	string res = "";
	if (dist == 0)
	{
	      res = "coincident";
	}
	else if (dist < 0.75)
	{
	      res = "near";
	}
	else if (dist < 1.5)
	{
	      res = "middle";
	}
	else
	{
	      res = "far";
	}
	
	return res;
}

string AngleDiffToText(float angle1, float angle2) // 1 to 2
{
	string anglestrset[8] = {"front", "front-left", "left", "left-back", "back", "back-right", "right", "right-front"};
	float diffAngle = angle1 - angle2 + 1*PI/8;
	while(diffAngle < 0)
	{
		diffAngle += 2 * PI;
	}
	int id = (int)(diffAngle / (PI/4));
	string res = anglestrset[id];
	return res;	
}

vector<string> DirectionToText(vector<float> direction)
{
	vector<string> res;
	vector<int> indecs = RankArray(direction);
	for (int i = 0; i < indecs.size(); i++)
	{
		if (indecs[i] == 3)
		{
			if (i == 0)
			{
				res.push_back("front");
			}
			if (i == 1)
			{
				res.push_back("left");
			}
			if (i == 2)
			{
				res.push_back("back");
			}
			if (i == 3)
			{
				res.push_back("right");
			}
		}
		if (indecs[i] == 2)
		{
			if (i == 0)
			{
				res.push_back("front");
			}
			if (i == 1)
			{
				res.push_back("left");
			}
			if (i == 2)
			{
				res.push_back("back");
			}
			if (i == 3)
			{
				res.push_back("right");
			}
		}
	}

	return res;
}

vector<string> HofToText(vector<float> hof)
{
	vector<float> direction = HofToDirection(hof);
	//printf("front: %f, left: %f, back: %f, right: %f\n", direction[0], direction[1], direction[2], direction[3]);
	vector<string> res = DirectionToText(direction);
	return res;
}

int HofToMainDirection8(vector<float> hof)
{
	//their are 8 direction from front with counter clock direction to left front (0 ~ 7)
	int res = 0;
	vector<float> direction = HofToDirection(hof);
	vector<int> indecs = RankArray(direction);
	int maindirt = 0;
	for (int i = 0; i < indecs.size(); i++)
	{
		if (indecs[i] == 3)
		{
			if (i == 0)
			{
				maindirt = 0;
				if (direction[1] > 0 && direction[3] > 0)
				{
					res = 0;
				}
				else if (direction[1] > 0)
				{
					res = 1;
				}
				else if (direction[3] > 0)
				{
					res = 7;
				}
			}
			if (i == 1)
			{
				maindirt = 2;
				if (direction[0] > 0 && direction[2] > 0)
				{
					res = 2;
				}
				else if (direction[0] > 0)
				{
					res = 1;
				}
				else if (direction[2] > 0)
				{
					res = 3;
				}
			}
			if (i == 2)
			{
				maindirt = 4;
				if (direction[1] > 0 && direction[3] > 0)
				{
					res = 4;
				}
				else if (direction[1] > 0)
				{
					res = 3;
				}
				else if (direction[3] > 0)
				{
					res = 5;
				}
			}
			if (i == 3)
			{
				maindirt = 6;
				if (direction[0] > 0 && direction[2] > 0)
				{
					res = 6;
				}
				else if (direction[0] > 0)
				{
					res = 7;
				}
				else if (direction[2] > 0)
				{
					res = 5;
				}
			}
		}
	}

	return res;
}

//////////////////////////////////////////////////////////////////////////
//build 4 vectors that store the weight of each piece for all four directions!
vector<vector<float> > BuildDirectionTable(vector<vector<float> > directions)
{
	vector<vector<float> > res;
	for (int i = 0; i < 4; i++)
	{
		vector<float> d; 
		for (int j = 0; j < directions.size(); j++)
		{
			d.push_back(directions[j][i]);
		}
		res.push_back(d);
	}
	return res;
}

int GetDirectionNumFromDirectionName(string directionName)
{
	int res = -1;
	if (directionName.compare("front") == 0)
	{
		res = 0;
	}
	if (directionName.compare("left") == 0)
	{
		res = 1;
	}
	if (directionName.compare("back") == 0)
	{
		res = 2;
	}
	if (directionName.compare("right") == 0)
	{
		res = 3;
	}
	return res;
}

float DiffAngle(float angle1, float angle2)
{
	float anglediff = angle2 - angle1;
	while (anglediff > PI)
	{
		anglediff -= 2 * PI;
	}
	while (anglediff < -PI)
	{
		anglediff += 2 * PI;
	}
	
	return anglediff;
}






#endif
