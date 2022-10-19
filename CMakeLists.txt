cmake_minimum_required(VERSION 3.10)
project(wolf_ros_node)

## Compile as C++14
add_compile_options(-std=c++14) 
# -fPIC and -rdynamic ensure unique singleton instance across shared libraries (for factories) see: https://stackoverflow.com/a/8626922
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -rdynamic")

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  tf_conversions
  tf2_ros
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(wolfcore REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LandmarkDetection.msg
  LandmarkDetectionArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
# produces the XXXConfig.cmake file among other things
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
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
  			    src/subscriber_landmarks.cpp
  			    src/subscriber_odom2d.cpp
  			    src/subscriber_pose.cpp)

# messages generated are a dependency of subscriber_landmarks
add_dependencies(subscriber_${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

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