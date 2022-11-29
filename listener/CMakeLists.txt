cmake_minimum_required(VERSION 2.8.3)
project(listener)
set(CMAKE_PREFIX_PATH _PATH "/opt/ros/melodic;${CMAKE_MODULE_PATH}")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation geometry_msgs)

add_service_files(
        FILES
        pointService.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

