#include "ros/ros.h"
#include "std_msgs/String.h"

#include "stdio.h"
#include "stdlib.h"
#include <cstdlib>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{

	// get the grouding types
	string rootDir = "/home/hri/hri_DATA/Targets/";
	vector<string> groundingList;
	vector<string> groundingDirList;
	
	DIR *dpdf;
	struct dirent *epdf;
	dpdf = opendir(rootDir.c_str());
	if (dpdf != NULL)
	{
		while (epdf = readdir(dpdf))
		{  
			//cout << epdf->d_name << endl;
			string dirName(epdf->d_name);
			if (dirName.size() > 5)
			{
				groundingDirList.push_back(rootDir + dirName + "/");
				groundingList.push_back(dirName);
				cout << dirName << endl;
			}
		}
	}

	return 0;
}