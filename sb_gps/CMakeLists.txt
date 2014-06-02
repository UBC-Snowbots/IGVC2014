cmake_minimum_required(VERSION 2.8.3)
project(sb_gps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sb_gps src/gps_data.cpp)
target_link_libraries(sb_gps ${catkin_LIBRARIES})

add_executable(sb_example src/gps_subscriber_example.cpp)
target_link_libraries(sb_example ${catkin_LIBRARIES})

## To add package paths:
## source ~/catkin_ws/devel/setup.bash
