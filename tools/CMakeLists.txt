cmake_minimum_required(VERSION 2.8)
project(tools)

find_package(PCL 1.8 REQUIRED PATHS /usr/local NO_DEFAULT_PAT)

set(PMR_INCLUDE_DIRS ../src)
include_directories(${PCL_INCLUDE_DIRS} ${PMR_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# pdc gen
set(PCD_GEN_SRC_LIST
    ../src/common/model_stl.cpp
    ../src/io/stl_reader.cpp
    ../src/conversion/stl2pcd.cpp
    pcd_generator.cpp
)
add_executable(pcd_gen ${PCD_GEN_SRC_LIST})
target_link_libraries(pcd_gen ${PCL_LIBRARIES})

# cloud transform
set(TRANSFORM_SRC_LIST 
    cloud_transform.cpp
)
add_executable(cloud_transform ${TRANSFORM_SRC_LIST})
target_link_libraries(cloud_transform ${PCL_LIBRARIES})

# pcd_viz
set(PCD_VIZ_SRC_LIST
    pcd_viz.cpp
)
add_executable(pcd_viz ${PCD_VIZ_SRC_LIST})
target_link_libraries(pcd_viz ${PCL_LIBRARIES})

# stl_viz
set(STL_VIZ_SRC_LIST
    ../src/visualization/stl_visualizer.cpp
    stl_viz.cpp
)
add_executable(stl_viz ${STL_VIZ_SRC_LIST})
target_link_libraries(stl_viz ${PCL_LIBRARIES})
