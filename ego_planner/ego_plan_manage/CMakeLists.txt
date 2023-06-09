cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  ego_plan_env
  ego_path_searching
  ego_bspline_opt
  ego_traj_utils
  message_generation
  cv_bridge
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  DataDisp.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS ego_plan_env ego_path_searching ego_bspline_opt ego_traj_utils message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
  )
add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(ego_traj_server src/traj_server.cpp)
target_link_libraries(ego_traj_server ${catkin_LIBRARIES})
add_dependencies(ego_traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(ego_traj_server_icuas src/traj_server_icuas.cpp)
target_link_libraries(ego_traj_server_icuas ${catkin_LIBRARIES})
add_dependencies(ego_traj_server_icuas ${${PROJECT_NAME}_EXPORTED_TARGETS})




