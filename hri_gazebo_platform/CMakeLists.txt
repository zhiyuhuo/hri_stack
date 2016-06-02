cmake_minimum_required(VERSION 2.8.3)
project(hri_gazebo_platform)

set(CMAKE_C11_FLAGS "${CMAKE_C11_FLAGS} ${GAZEBO_C11_FLAGS}")
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(hri_gazebo_platform_kinect_tilt_plugin src/kinect_tilt_plugin.cpp)
target_link_libraries(hri_gazebo_platform_kinect_tilt_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_library(hri_gazebo_platform_wireless_receiver_plugin src/wireless_receiver_plugin.cpp)
# target_link_libraries(hri_gazebo_platform_wireless_receiver_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
