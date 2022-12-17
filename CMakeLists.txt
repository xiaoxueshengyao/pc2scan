cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_to_scan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs

)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pointcloud_to_scan
    CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_to_scan src/pctoscan.cpp)
target_link_libraries(pointcloud_to_scan ${catkin_LIBRARIES})