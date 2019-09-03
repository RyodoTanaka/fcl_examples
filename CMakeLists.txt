cmake_minimum_required(VERSION 2.8.3)
project(fcl_examples)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  )
# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# FCL
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
find_package(FCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########
set(SOURCE
  sphere_sphere
  #box_sphere
  )
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  )
link_directories(
  ${FCL_LIBRARY_DIRS}
)

foreach(src ${SOURCE})
  add_executable(${src} src/${src}.cpp)
  target_link_libraries(${src} ${FCL_LIBRARIES} ${catkin_LIBRARIES} )
endforeach(src)
