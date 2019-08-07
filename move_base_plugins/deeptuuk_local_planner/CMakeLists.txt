cmake_minimum_required(VERSION 2.8.3)
project(deeptuuk_local_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(include)
add_library (deeptuuk_local_planner src/local_planner.cpp)