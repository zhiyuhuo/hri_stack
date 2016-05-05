#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "hri_robot/Robot/Header.h"

int main(int argc, char **argv)
{
	//
	Robot _robot;
	
	if (argc > 1)
	{
	      string targetObject(argv[1]);
	      _robot.m_targetObject = targetObject;
	}
	
	_robot.ConnectToServer();
	_robot.KeyboardControlForLanguageGeneration();

	return 0;
}