cmake_minimum_required(VERSION 2.8.3)
project(lidar_preprocess)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  message_generation
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  gps_ins_msg.msg
  gpgga_msg.msg
  odom_and_status.msg
  gps_status.msg
  odom_and_ctrl.msg
  headinga_msg.msg
  cloud_msgs.msg
  is_intersection.msg
  corner_label.msg
  odom_optimize_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_integration
#  CATKIN_DEPENDS pcl_ros roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
${GTSAM_INCLUDE_DIR}
)
link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)
add_library(${PROJECT_NAME}-points-cluster
  src/dbscan.cpp)
add_executable(${PROJECT_NAME}_filter_process src/filter_process.cpp)
add_dependencies(${PROJECT_NAME}_filter_process ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}-points-cluster)
target_link_libraries(${PROJECT_NAME}_filter_process 
  ${PROJECT_NAME}-points-cluster
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(${PROJECT_NAME}_show src/pcl_show.cpp)
add_dependencies(${PROJECT_NAME}_show ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}-points-cluster)
target_link_libraries(${PROJECT_NAME}_show 
  ${PROJECT_NAME}-points-cluster
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES})



































