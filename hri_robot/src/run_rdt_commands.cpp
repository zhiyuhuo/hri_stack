#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

#include <sstream>
#include "hri_robot/Robot/Header.h"

using namespace std;

int main(int argc, char **argv)
{
	//
	if (argc <= 1)
	{
		cout << "error! no room obj name!\n";
		return -1;
	}
	string roomobjidstr(argv[1]);
	cout << "room obj: " << roomobjidstr << endl;
	string fileName = "/home/hri/hri_DATA/new_three_worlds_grounding_res/" + roomobjidstr + ".txt";
	cout << fileName << endl;
	string formattedCmdStr = "";
	string rawCmdStr = "";
	ifstream myfile (fileName.c_str());
	if (myfile.is_open())
	{
		getline (myfile, formattedCmdStr);
		getline (myfile, rawCmdStr);
		cout << rawCmdStr << endl;
	}
	else	
	{		
		cout << "no file found!\n";
		return 0;
	}
	
	string groundingstr = "";
	std::vector<string> groundings;
	for (int i = 0; i < formattedCmdStr.size(); i++)
	{
		if (formattedCmdStr[i] != ',')
		{
			groundingstr.push_back(formattedCmdStr[i]);
		}
		else
		{
			groundings.push_back(groundingstr);
			groundingstr.clear();
		}
	}
	
	
	Robot _robot;
	_robot.ConnectToServer();
	_robot.TestFromRDT(groundings);

	return 0;
}
