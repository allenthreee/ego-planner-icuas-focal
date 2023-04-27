cmake_minimum_required(VERSION 2.8.3)
project(icuas_misc)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    message_generation
    icuas23_competition
    ego_plan_env
    cv_bridge
    tf
)

add_message_files(
  FILES
  blank.msg
  )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
#  DEPENDS system_lib
  CATKIN_DEPENDS ego_plan_env message_runtime
  # geometry_msgs 
  # mavros_msgs 
  # roscpp
  # std_msgs
  # message_generation
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

catkin_install_python(PROGRAMS 
  scripts/talker.py
  scripts/tsp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(a_fsm_eg src/a_fsm_eg.cpp)
target_link_libraries(a_fsm_eg ${catkin_LIBRARIES})

add_executable(a_fsm_node 
  src/a_fsm_node.cpp
  src/a_fsm.cpp)
target_link_libraries(a_fsm_node ${catkin_LIBRARIES})

add_executable(a_fsm_vexp
  src/a_fsm_vexp.cpp
  )
target_link_libraries(a_fsm_vexp ${catkin_LIBRARIES})


add_executable(offb
  src/offb.cpp)
target_link_libraries(offb ${catkin_LIBRARIES})


