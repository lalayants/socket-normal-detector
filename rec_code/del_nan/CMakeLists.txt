cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcl-del_nan)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (del_nan del_nan.cpp)
target_link_libraries (del_nan ${PCL_LIBRARIES})