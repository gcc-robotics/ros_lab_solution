cmake_minimum_required(VERSION 2.8.3)
project(line_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES line_follower`
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_library(line_follower`
   src/main.cpp
 )

add_executable(line_follower_node src/main.cpp)

add_dependencies(line_follower_node line_follower_generate_messages_cpp)

target_link_libraries(line_follower_node ${catkin_LIBRARIES})