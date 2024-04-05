cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  message_generation
  nav_msgs
  pcl_ros
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

find_package(PCL REQUIRED)


catkin_package(
  INCLUDE_DIRS include 
#  LIBRARIES a_loam
CATKIN_DEPENDS roscpp rospy geometry_msgs nav_msgs std_msgs
  DEPENDS EIGEN3 PCL

)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
)


# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/scanRegistration.cpp)


# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

