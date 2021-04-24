cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar_planner)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_COMPILER "g++")

# add_subdirectory(test_the_plugin)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pluginlib
  costmap_2d
  roscpp
  nav_core
  tf2_ros
  ompl
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  geometry_msgs
  pluginlib
  roscpp
  nav_core
  tf2_ros
  ompl
)

include_directories(
  include
  test_the_plugin/include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# find_package(ompl REQUIRED)

# if(NOT OMPL_FOUND)
#     message(AUTHOR_WARNING,"Open Motion Planning Library not found")
# endif(NOT OMPL_FOUND)


add_executable(test_planner test_the_plugin/src/test.cpp test_the_plugin/src/test_plugins.cpp)
target_link_libraries(test_planner ${catkin_LIBRARIES})

add_executable(tf_test_broadcaster test_the_plugin/src/tf_broadcaster.cpp)
target_link_libraries(tf_test_broadcaster ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
  src/planner_core.cpp 
  src/hybrid_astar.cpp 
  src/node2d.cpp
  src/a_start.cpp
  src/visualize.cpp
  src/node3d.cpp
  src/hybrid_astar.cpp
  src/algorithm.cpp
  src/dubins.cpp
  src/ReedsShepp.cpp
)
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)