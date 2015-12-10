#ifndef BUILDLOCALMAP_H
#define BUILDLOCALMAP_H
#include <stdio.h>
#include <math.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Header.h"

using namespace std;

void BuildLocalMap(float points[], IplImage* localMap)
{
	int width = localMap->width;
	int height = localMap->height;
	int i;	
	for(i = 0; i < LOCALMAP_SIZE; i++)
	{
		localMap->imageData[i] = 0;
	}
	
	for(i = 0; i < IMG_SIZE; i++)
	{
		float x, y, z;
		x = points[3*i];
		y = points[3*i+1];
		z = points[3*i+2];
		x = x + LOCALMAP_X / 2;	

		int xx, yy;
		xx = x / LOCALMAP_RESOLUTION;
		yy = y / LOCALMAP_RESOLUTION;

		if (xx<width && xx>=0 && yy<height && yy>=0 && z > 0.1)
		{
			int gi = width * yy + xx;
			localMap->imageData[gi] = 255; 
		}
	}
	
}

void BuildLocalMap(vector<float> points, IplImage* localMap)
{
	int width = localMap->width;
	int height = localMap->height;
	int i;	
	for(i = 0; i < LOCALMAP_SIZE; i++)
	{
		localMap->imageData[i] = 0;
	}
	
	for(i = 0; i < points.size()/3; i++)
	{
		float x, y, z;
		x = points[3*i];
		y = points[3*i+1];
		z = points[3*i+2];
		x = x + LOCALMAP_X / 2;	

		int xx, yy;
		xx = x / LOCALMAP_RESOLUTION;
		yy = y / LOCALMAP_RESOLUTION;

		if (x == x && y == y && z == z)
		{
			if (xx<width && xx>=0 && yy<height && yy>=0 && z > 0.1)
			{
				int gi = width * yy + xx;
				localMap->imageData[gi] = 255;
			}
		}
	}
	
}

void BuildLocalMap(vector<float> points, Mat localMap)
{
	int width = localMap.cols;
	int height = localMap.rows;
	int i;	
	for(i = 0; i < width*height; i++)
	{
		localMap.data[i] = 0;
	}
	
	for(i = 0; i < points.size()/3; i++)
	{
		float x, y, z;
		x = points[3*i];
		y = points[3*i+1];
		z = points[3*i+2];
		x = x + LOCALMAP_X / 2;	

		int xx, yy;
		xx = x / LOCALMAP_RESOLUTION;
		yy = y / LOCALMAP_RESOLUTION;

		if (x == x && y == y && z == z)
		{
			if (xx<width && xx>=0 && yy<height && yy>=0 && z > 0.1)
			{
				int gi = width * yy + xx;
				localMap.data[gi] = 255;
			}
		}
	}
	
}

float FindDistance(int x, int y)
{
	float res = 0;
	res = sqrt((x - LOCALMAP_WIDTH/2 + 0.5)*(x - LOCALMAP_WIDTH/2 + 0.5)+y*y);

	return res;
}

float FindBlobDistance(int id, IplImage* Ilabel)
{
	int width = Ilabel->width;
	int height = Ilabel->height;
	unsigned char iduc = (unsigned char)id;

	float distance = 0;
	int total = 0;

	for(int i = 0; i < Ilabel->imageSize; i++)
	{
		if(Ilabel->imageData[i] == iduc)
		{
			distance += FindDistance(i%width, i/width);
			total++;
		}
	}
	distance = distance / total;

	return distance;
}

float FindBlobDistance(int id, cv::Mat Ilabel)
{
	int width = Ilabel.cols;
	int height = Ilabel.rows;
	unsigned char iduc = (unsigned char)id;

	float distance = 0;
	int total = 0;

	for(int i = 0; i < width*height; i++)
	{
		if(Ilabel.data[i] == iduc)
		{
			distance += FindDistance(i%width, i/width);
			total++;
		}
	}
	distance = distance / total;

	return distance;
}

int FindTheNearestBlob(IplImage* Ilabel)
{	
	int width = Ilabel->width;
	int height = Ilabel->height;
	
	int numblob = 0;
	for (int i = 0; i < Ilabel->imageSize; i++)
	{
		int idi = (int)Ilabel->imageData[i];
		if (idi > numblob)
		{
			numblob = idi;
		}

	}
	float mindistance = Ilabel->imageSize;
	int minid = 1;
	
	for(int i = 1; i <= numblob; i++)
	{
		float dist = FindBlobDistance(i, Ilabel);
		if (dist < mindistance) 
		{
			mindistance = dist;
			minid = i;
		}
	}

	return minid;	
}

#endif
