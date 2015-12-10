#ifndef BWLABEL_H
#define BWLABEL_H
#include "stdio.h"
#include "stdlib.h"
//#include <apvector>	
#include <vector>
#include <map>

#include <math.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Header.h"

using namespace std;
using namespace cv;

void BwLabel(IplImage* I, IplImage* Ilabel)
{	
	int width = I->width;
	int height = I->height;
	
	unsigned char *img = (unsigned char *)malloc(width*height);
	memcpy(img, I->imageData, width*height);

	unsigned int *out = (unsigned int *)malloc(sizeof(*out)*width*height);
	ConnectedComponents cc(30);
    	cc.connected(img, out, width, height,
		 std::equal_to<unsigned char>(),
		 constant<bool,true>());
	
    	unsigned char *out_uc = (unsigned char *)malloc(width*height);
    	std::copy(out, out+width*height, out_uc);

	memcpy(Ilabel->imageData, out_uc, width*height);

}

void BwLabel(Mat I, Mat Ilabel)
{	
	int width = I.cols;
	int height = I.rows;
	
	unsigned char *img = (unsigned char *)malloc(width*height);
	memcpy(img, I.data, width*height);

	unsigned int *out = (unsigned int *)malloc(sizeof(*out)*width*height);
	ConnectedComponents cc(30);
    	cc.connected(img, out, width, height,
		 std::equal_to<unsigned char>(),
		 constant<bool,true>());
	
    	unsigned char *out_uc = (unsigned char *)malloc(width*height);
    	std::copy(out, out+width*height, out_uc);

	memcpy(Ilabel.data, out_uc, width*height);

}

void SetBwMapBackgroundZero(Mat localMapLabel)
{
	int width = localMapLabel.cols;
	int height = localMapLabel.rows;
	int tail1 = 0;
	int tail2 = width-1;
	int mapsize = width*height;

	if (localMapLabel.data[tail1] > 0 || localMapLabel.data[tail2] > 0)
	{
		unsigned char label = localMapLabel.data[tail1];
		for (int i = 0; i < mapsize; i++)
		{
			if (localMapLabel.data[i] == label)
			{
				localMapLabel.data[i] = (unsigned char)(0);
			}
			else if (localMapLabel.data[i] == (unsigned char)(0))
			{
				localMapLabel.data[i] = label;
			}
		}
	}
}

#endif










