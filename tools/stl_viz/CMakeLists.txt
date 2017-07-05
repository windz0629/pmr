cmake_minimum_required(VERSION 2.8)
project(pcd_gen)

find_package(PCL 1.7 REQUIRED PATHS /usr NO_DEFAULT_PATH)

set(PMR_INCLUDE_DIRS ../../src)
include_directories(${PCL_INCLUDE_DIRS} ${PMR_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(SRC_LIST 
../../src/visualization/stl_visualizer.cpp 
src/stl_viz.cpp
)

add_executable(stl_viz ${SRC_LIST})
target_link_libraries(stl_viz ${PCL_LIBRARIES})
