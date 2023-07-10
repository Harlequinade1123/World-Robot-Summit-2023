cmake_minimum_required(VERSION 3.0.2)
project(wrs_motion_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wrs_motion_planner
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)
find_package (Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/wrs_motion_planner_node.cpp src/kinematics.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen)

add_executable(${PROJECT_NAME}_init_node src/wrs_initial_motion.cpp src/wrs_motion_planner_node.cpp src/kinematics.cpp)
target_link_libraries(${PROJECT_NAME}_init_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_init_node Eigen3::Eigen)