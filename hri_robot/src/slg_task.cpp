#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "hri_robot/Robot/Header.h"

int main(int argc, char **argv)
{
	//
	Robot _robot;
	if (argc > 2)
	{
	      string worldName(argv[1]);
	      string targetObject(argv[2]);
	      _robot.ConnectToServer();
	      _robot.KeyboardControlForLanguageGeneration(worldName, targetObject);
	}
	else
	{
	      cout << "incorrected input" << endl;
	}

	return 0;
}