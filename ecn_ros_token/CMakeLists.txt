cmake_minimum_required(VERSION 2.8.3)
project(ecn_ros_token)


add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)


add_executable(testtoken src/example.cpp include/ecn_ros_token/ecn_token.h)
target_link_libraries(testtoken ${catkin_LIBRARIES})
