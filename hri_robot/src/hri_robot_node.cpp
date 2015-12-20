#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "hri_robot/Robot/Header.h"

int main(int argc, char **argv)
{
	//
	Robot _robot;
	_robot.RunNode();

	return 0;
}