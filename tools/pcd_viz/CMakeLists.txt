cmake_minimum_required(VERSION 2.8)
project(pcd_viz)

find_package(PCL 1.7 REQUIRED PATHS /usr NO_DEFAULT_PATH)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(SRC_LIST 
src/pcd_viz.cpp
)

add_executable(pcd_viz ${SRC_LIST})
target_link_libraries(pcd_viz ${PCL_LIBRARIES})
