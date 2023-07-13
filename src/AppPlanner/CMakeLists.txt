cmake_minimum_required(VERSION 3.0.2)
project(app_hnu_planner)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost COMPONENTS iostreams system filesystem)

find_package(
  catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  rospy
  visualization_msgs
  sandbox_msgs
  tf
  std_msgs
  nav_msgs
)


SET(CMAKE_CXX_FLAGS "-DROS ${CMAKE_CXX_FLAGS}")
# SET(CMAKE_BUILD_TYPE "DEBUG")
# SET(CMAKE_BUILD_TYPE "RELEASE")
# add_compile_options(-O3)

catkin_package(
 INCLUDE_DIRS .
 CATKIN_DEPENDS roscpp std_msgs message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ./
  /usr/include/eigen3/
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/multi_vehicle_experiments.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(app_hnu_planner
  common/math/aabox2d.cc
  common/math/box2d.cc
  common/math/line_segment2d.cc
  common/math/linear_interpolation.cc
  common/math/math_utils.cc
  common/math/polygon2d.cc
  common/math/trajectory1d.cc
  common/math/vec2d.cc
  common/math/quintic_polynomial_curve1d.cc
  common/math/angle.cc
  common/math/sin_table.cc
  common/math/integral.cc
  common/math/quintic_spiral_path.cc
  common/data/discretized_trajectory.cc
  common/util/point_cloud.cc
  common/util/color.cc
  common/util/file_utils.cc
  common/visualization_plot.cpp
  common/environment.cc
  common/frame.cc
  common/obstacle.cc
  map/map_generator.cc
  planning/decider/dp_path_decider.cc
  planning/app_planner.cc
  node.cpp
)

target_include_directories(app_hnu_planner PUBLIC
  ./
  third_party/nlohmann/
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(app_hnu_planner
  ${catkin_LIBRARIES}
)