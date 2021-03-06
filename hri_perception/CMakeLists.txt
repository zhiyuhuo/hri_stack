cmake_minimum_required(VERSION 2.8.3)
project(hri_perception)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospack
  rospy
  sensor_msgs
  std_msgs
  nav_msgs 
  geometry_msgs
  message_generation
  tf
)

find_package(PCL 1.2 REQUIRED)

add_message_files(
  FILES
  Env.msg
)

add_service_files(
  FILES
  Perception.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hri_perception2015a
 CATKIN_DEPENDS cv_bridge image_transport pcl_conversions pcl_ros roscpp rospack rospy sensor_msgs std_msgs nav_msgs geometry_msgs message_runtime tf
#  DEPENDS system_lib
)	

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (build_semantic_map src/build_semantic_map.cpp)
target_link_libraries (build_semantic_map ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})	

add_executable (hri_perception_node src/hri_perception_node.cpp)
target_link_libraries (hri_perception_node ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})	
add_dependencies(hri_perception_node ${hri_perception_EXPORTED_TARGETS})

add_executable (hri_perception_reconstruction src/hri_perception_reconstruction.cpp)
target_link_libraries (hri_perception_reconstruction ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})	
add_dependencies(hri_perception_reconstruction ${hri_perception_EXPORTED_TARGETS})

add_executable (hri_perception_server src/hri_perception_server.cpp)
target_link_libraries (hri_perception_server ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hri_perception_server ${hri_perception_EXPORTED_TARGETS})	

add_executable(hri_perception_test src/hri_perception_test.cpp)
target_link_libraries(hri_perception_test ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable (cloud_viewer src/cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})	

add_executable (get_pcd_sample_normal src/get_pcd_sample_normal.cpp)
target_link_libraries (get_pcd_sample_normal ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable (get_pcd_sample_fpfh src/get_pcd_sample_fpfh.cpp)
target_link_libraries (get_pcd_sample_fpfh ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable (get_pcd_sample_narf src/get_pcd_sample_narf.cpp)
target_link_libraries (get_pcd_sample_narf ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})


