cmake_minimum_required(VERSION 3.0.2)
project(viewer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_filters
  interactive_markers
  visualization_msgs
  tf
  my_msg
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES viewer
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS
  interactive_markers
  roscpp
  visualization_msgs
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(viewer
  src/viewer.cpp
)
add_dependencies(viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viewer  ${catkin_LIBRARIES})

