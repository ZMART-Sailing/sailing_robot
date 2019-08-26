cmake_minimum_required(VERSION 2.8.3)
project(ball_detect)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    cv_bridge
    image_transport
    message_generation
    darknet_ros_msgs
)

find_package(OpenCV REQUIRED)
find_package(realsense2)

add_message_files(
    FILES
	BoundingBox.msg
	BoundingBoxes.msg
  BoatAndBall.msg
)

generate_messages(
    DEPENDENCIES
	std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cam_detect
  CATKIN_DEPENDS rospy roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${realsense_INCLUDE_DIR}
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ball_detect.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(color_detect src/color_detect_node.cpp)
 add_executable(yolo_detect src/yolo_detect_node.cpp)
 add_executable(sim_gps src/simulator_gps.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(color_detect
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY}
 )
 target_link_libraries(yolo_detect
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY}
 )
 target_link_libraries(sim_gps
  ${catkin_LIBRARIES}
)


