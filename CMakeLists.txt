cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_icp)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (interactive_icp interactive_icp.cpp)
target_link_libraries (interactive_icp ${PCL_LIBRARIES})

add_executable (draw_model draw_model.cpp)
target_link_libraries (draw_model ${PCL_LIBRARIES})

add_executable (extract_indices extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})

add_executable (statistical_removal statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})
