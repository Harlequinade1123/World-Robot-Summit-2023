cmake_minimum_required(VERSION 3.0.2)
project(wrs_robot_viz)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
link_libraries(${OPENGL_LIBRARIES})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
link_libraries(${GLEW_LIBRARIES})

find_package(Threads REQUIRED)

find_package (Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_viz
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/wrs_robot_viz_node.cpp src/psketch.cpp src/sketch.cpp src/kinematics.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node glfw OpenGL::GL)
target_link_libraries(${PROJECT_NAME}_node Threads::Threads)
target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen)