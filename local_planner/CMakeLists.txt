cmake_minimum_required(VERSION 3.5)
project(local_planner)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic -O2)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mmr_base REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(local_planner_node
	src/main.cpp
	src/local_planner/generic_planner.cpp
	src/local_planner/acceleration_planner.cpp
	src/local_planner/skidpad_planner.cpp
	src/local_planner/autocross_planner.cpp
	src/local_planner/utils.cpp
)

target_include_directories(local_planner_node PUBLIC include)
set_property(TARGET local_planner_node PROPERTY CXX_STANDARD 20)
ament_target_dependencies(local_planner_node yaml-cpp ament_index_cpp rclcpp geometry_msgs nav_msgs visualization_msgs mmr_base)
target_link_libraries(local_planner_node yaml-cpp)

install(TARGETS 
		local_planner_node
  		DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
	    launch
	    DESTINATION share/${PROJECT_NAME}/)

ament_package()