cmake_minimum_required(VERSION 2.8.3)
project(fcl_examples)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  fcl
  )
# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  CATKIN_DEPENDS roscpp fcl
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(sphere_sphere src/sphere_sphere.cpp)
target_link_libraries(sphere_sphere
  ${catkin_LIBRARIES}
)
