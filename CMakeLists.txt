cmake_minimum_required(VERSION 2.8.3)
project(rov_visp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  image_transport
  cv_bridge
  visp_bridge
  tf
  tf2_ros
  tf_conversions
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_vis
  CATKIN_DEPENDS 
    roscpp
    geometry_msgs
    image_transport
    cv_bridge
    visp_bridge
    tf
    tf2_ros
    tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

## Declare a cpp executable
add_executable(main src/visual_pose_servo.cpp)
target_link_libraries(main ${catkin_LIBRARIES})

add_executable(tag_vis src/visp_feature.cpp)
target_link_libraries(tag_vis ${catkin_LIBRARIES})