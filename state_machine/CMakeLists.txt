cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES state_machine
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_library(state_machine
   src/main.cpp
 )

add_executable(state_machine_node src/main.cpp)

add_dependencies(state_machine_node state_machine_generate_messages_cpp)

target_link_libraries(state_machine_node ${catkin_LIBRARIES})