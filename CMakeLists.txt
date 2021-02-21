cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar_planner)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_COMPILER "g++")

add_subdirectory(test_the_plugin)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  costmap_2d
  roscpp
  nav_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  pluginlib
  roscpp
  nav_core
)

include_directories(
  include
  test_the_plugin/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_planner test_the_plugin/src/test.cpp test_the_plugin/src/test_plugins.cpp)
target_link_libraries(test_planner ${catkin_LIBRARIES})

add_executable(tf_test_broadcaster test_the_plugin/src/tf_broadcaster.cpp)
target_link_libraries(tf_test_broadcaster ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
  src/planner_core.cpp 
  src/hybrid_astar.cpp 
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)