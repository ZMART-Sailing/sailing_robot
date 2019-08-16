cmake_minimum_required(VERSION 2.8.3)

project(correction_receiver_ros)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

catkin_package(
    INCLUDE_DIRS include
)

add_executable(correction_receiver_ros_node src/correction_receiver_ros_node.cpp src/correction_receiver_ros.cpp)

target_link_libraries(correction_receiver_ros_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
