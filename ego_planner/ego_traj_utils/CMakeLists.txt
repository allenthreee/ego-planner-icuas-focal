cmake_minimum_required(VERSION 2.8.3)
project(ego_traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  ego_bspline_opt
  ego_path_searching
  roscpp
  std_msgs
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_traj_utils
 CATKIN_DEPENDS ego_path_searching ego_bspline_opt
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( ego_traj_utils
    src/planning_visualization.cpp 
    src/polynomial_traj.cpp 
    )
target_link_libraries( ego_traj_utils
    ${catkin_LIBRARIES} 
    )  
