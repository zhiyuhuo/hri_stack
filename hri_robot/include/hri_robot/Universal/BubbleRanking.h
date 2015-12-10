#ifndef TOOLFUNCTIONS_BUBBLERANKING_H
#define TOOLFUNCTIONS_BUBBLERANKING_H

#include "stdio.h"
#include "stdlib.h"
#include "std_msgs/String.h"

#include <math.h>
#include <time.h>
#include <vector>

#include "Header.h"

std::vector<int> bubble_rank(std::vector<float> A)
{
	std::vector<int> B;
	for(int i=0; i<A.size(); i++)
	{
		B.push_back(i);
	}
	bool swapped = false;
	do
	{
		swapped = false;
	    for(int i=0; i<A.size()-1; i++)
		{
			if(A[B[i]] > A[B[i+1]])
			{
				int temp = B[i];
				B[i] = B[i+1];
				B[i+1] = temp;
				swapped = true;
	      	}
		}
	}
	while(swapped);

	return B;
}

#endif 
