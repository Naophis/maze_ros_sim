cmake_minimum_required(VERSION 3.0.2)
project(maze_solver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_filters
  my_msg
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(maze_solver
  src/controller.cpp
  src/logic.cpp
  src/adachi.cpp
)
add_dependencies(maze_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(maze_solver ${catkin_LIBRARIES})

add_executable(path_creator
  src/path_controller.cpp
  src/path_creator.cpp
  src/logic.cpp
)
add_dependencies(path_creator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_creator ${catkin_LIBRARIES})

