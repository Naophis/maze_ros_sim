cmake_minimum_required(VERSION 3.0.2)
project(my_msg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  maze.msg
  ego.msg
  base_path_element.msg
  base_path.msg
  path.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(

)