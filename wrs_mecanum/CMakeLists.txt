cmake_minimum_required(VERSION 3.0.2)
project(wrs_mecanum)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  dynamixel_sdk
)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
link_libraries(${OPENGL_LIBRARIES})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
link_libraries(${GLEW_LIBRARIES})

find_package(Threads REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  dynamixel_sdk
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/wrs_mecanum_node.cpp src/psketch.cpp src/sketch.cpp src/kinematics.cpp src/Dynamixel.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_node glfw OpenGL::GL)
target_link_libraries(${PROJECT_NAME}_node Threads::Threads)