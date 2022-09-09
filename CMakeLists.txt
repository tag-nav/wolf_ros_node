cmake_minimum_required(VERSION 3.10)
project(wolf_ros_node)

## Compile as C++14
add_compile_options(-std=c++14) 
# -fPIC and -rdynamic ensure unique singleton instance across shared libraries (for factories) see: https://stackoverflow.com/a/8626922
SET(CMAKE_CXX_FLAGS "-fPIC -rdynamic")

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  tf_conversions
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(wolfcore REQUIRED)


###################################
## catkin specific configuration ##
###################################
# produces the XXXConfig.cmake file among other things
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/node.cpp)

add_library(subscriber_${PROJECT_NAME}
  			    src/subscriber_diffdrive.cpp
  			    src/subscriber_odom2d.cpp
  			    src/subscriber_pose.cpp)

add_library(publisher_${PROJECT_NAME}
  			    src/publisher_graph.cpp
  			    src/publisher_pose.cpp
  			    src/publisher_state_block.cpp
  			    src/publisher_tf.cpp
  			    src/publisher_trajectory.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(subscriber_${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      wolfcore
                      )

target_link_libraries(publisher_${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      wolfcore
                      )

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      wolfcore
                      )