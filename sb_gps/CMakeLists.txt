cmake_minimum_required(VERSION 2.8.3)
project(sb_gps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sb_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs sb_msgs
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(sb_gps src/final.cpp)
target_link_libraries(sb_gps ${catkin_LIBRARIES})

add_executable(sb_sub src/gps_subscriber_example.cpp)
target_link_libraries(sb_sub ${catkin_LIBRARIES})

add_executable(sb_data src/sample_gps_output.cpp)
target_link_libraries(sb_data ${catkin_LIBRARIES})

## To add package paths:
## source ~/catkin_ws/devel/setup.bash
