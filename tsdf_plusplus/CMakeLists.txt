cmake_minimum_required(VERSION 2.8.3)
project(tsdf_plusplus)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread filesystem)

set(CMAKE_CXX_FLAGS "-std=c++14")

cs_add_library(${PROJECT_NAME}
  src/alignment/icp.cc
  src/core/object_volume.cc
  src/core/map.cc
  src/core/segment.cc
  src/integrator/integrator.cc
  src/mesh/color_map.cc
  src/mesh/mesh_integrator.cc
  src/visualizer/visualizer.cc
)



cs_install()
cs_export()
