cmake_minimum_required(VERSION 2.8)
project(cloud_transf)

find_package(PCL 1.7 REQUIRED PATHS /usr NO_DEFAULT_PATH)

set(PMR_INCLUDE_DIRS ../../src)
include_directories(${PCL_INCLUDE_DIRS} ${PMR_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(SRC_LIST 
src/cloud_transform.cpp
)

add_executable(cloud_transform ${SRC_LIST})
target_link_libraries(cloud_transform ${PCL_LIBRARIES})
