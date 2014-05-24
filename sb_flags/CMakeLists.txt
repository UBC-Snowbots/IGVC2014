cmake_minimum_required(VERSION 2.8)
project( Sight )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( Sight main.cpp )
target_link_libraries( Sight ${OpenCV_LIBS} )

