cmake_minimum_required(VERSION 2.8.3)
project(target_generator)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(Node src/Node.cpp)
target_link_libraries(Node ${catkin_LIBRARIES})

add_library(AStar src/AStar.cpp)
target_link_libraries(AStar ${catkin_LIBRARIES} Node)

add_library(Firefly src/Firefly.cpp)
target_link_libraries(Firefly ${catkin_LIBRARIES})

add_library(TargetGenerator src/TargetGenerator.cpp)
target_link_libraries(TargetGenerator ${catkin_LIBRARIES} AStar Firefly)

add_executable(target_generator src/main.cpp)
target_link_libraries(target_generator ${catkin_LIBRARIES} TargetGenerator)
#add_dependencies(target_generator target_generator_generate_messages_cpp)
