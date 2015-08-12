cmake_minimum_required(VERSION 2.8.3)
project(irobot_ir_calibrate)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
)

catkin_package(
	INCLUDE_DIRS src
	LIBRARIES irobot_ir_calibrate
	CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_library(irobot_ir_calibrate
	src/main.cpp
)

add_executable(irobot_ir_calibrate_node src/main.cpp)

add_dependencies(irobot_ir_calibrate_node irobot_ir_calibrate_generate_messages_cpp)

target_link_libraries(irobot_ir_calibrate_node mgl ${catkin_LIBRARIES})