#ifndef APPENDIXMATH_H
#define APPENDIXMATH_H

#include "stdio.h"
#include "stdlib.h"

#include <vector>
#include <iostream>
#include <math.h>
#include <time.h>

#include "Header.h"

using namespace std;
//////////////////////////////////////////////////////////////////////////
//Array ranking, return index from large to small
vector<int> RankArray(vector<float> table)
{
	int tableLength = table.size();
	vector<int> ranklabel(tableLength, 0);
	vector<int> result(tableLength, 0);
	for(int i=0; i < tableLength; i++)
	{
		int rnk=0;
		for(int z=0; z< tableLength; z++)
		{
			if(table[z]<table[i])
			{
				rnk++;
			}
		}
		ranklabel[i]=rnk;
	}
	for(int i=0; i < tableLength; i++)
	{
		result[tableLength - 1 - ranklabel[i]] = i;
	}
	return result;
}

vector<int> RankArray(vector<int> table)
{
	int tableLength = table.size();
	vector<int> ranklabel(tableLength, 0);
	vector<int> result(tableLength, 0);
	for(int i=0; i < tableLength; i++)
	{
		int rnk=0;
		for(int z=0; z< tableLength; z++)
		{
			if(table[z]<table[i])
			{
				rnk++;
			}
		}
		ranklabel[i]=rnk;
	}
	for(int i=0; i < tableLength; i++)
	{
		result[tableLength - 1 - ranklabel[i]] = i;
	}
	return result;
}

float Gaussian(float u, float d, float x)
{
	float res;
	//res = 1 / sqrt(2 * PI * pow(d, 2)) * exp(-0.5 * pow(x - u, 2) / pow(d, 2));
	res = exp(-0.5 * pow(x - u, 2) / pow(d, 2));
	return res;
}

#endif