#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <stdio.h>
#include <math.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Header.h"

using namespace std;

vector<float> Transformation(vector<float> src, float height, float angle)
{
	vector<float> dst(IMG_SIZE3, 0);
	angle = angle * PI / 180;

	for(int i = 0; i < IMG_SIZE; i++)
	{
		float x0, y0, z0;
		float x, y, z;

		x0 = src[3*i];
		y0 = src[3*i+2];
		z0 = -src[3*i+1];

		x = x0;
		y = cos(angle)*y0 + sin(angle)*z0;
		z = -sin(angle)*y0 + cos(angle)*z0 + height;

		dst[3*i] = x;
		dst[3*i+1] = y;
		dst[3*i+2] = z;
	}
	return dst;

}

#endif
