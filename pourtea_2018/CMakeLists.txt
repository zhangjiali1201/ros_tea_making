cmake_minimum_required(VERSION 2.8.3)
project(pourtea_2018)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  topic_tools
  sound_play
	tf
)

add_definitions(-std=c++11)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 2017kunshan
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)




## Declare a C++ executable

add_executable(voice_analysis
  src/voice_analysis.cpp
)
add_dependencies(voice_analysis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(voice_analysis
  ${catkin_LIBRARIES}
)
