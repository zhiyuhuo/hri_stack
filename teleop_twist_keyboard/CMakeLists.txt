cmake_minimum_required(VERSION 2.8.3)
project(teleop_twist_keyboard)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospack
  rospy
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hri_perception2015a
 CATKIN_DEPENDS rospack rospy geometry_msgs std_msgs 
#  DEPENDS system_lib
)	


	


