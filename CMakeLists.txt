cmake_minimum_required(VERSION 2.8.3)
project(fcl_examples)

####################
## compile option ##
####################
add_compile_options(-std=c++17)

#####################
## catkin settings ##
#####################
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
catkin_package(
  CATKIN_DEPENDS roscpp
)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

###########
## Eigen ##
###########
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(${EIGEN3_LIBRARY_DIRS})

#########
## FCL ##
#########
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
find_package(FCL REQUIRED)
include_directories(${FCL_INCLUDE_DIRS})
link_directories(${FCL_LIBRARY_DIRS})

###########
## Build ##
###########
set(SOURCE
  sphere_sphere
  box_sphere
  box_box
  )

foreach(src ${SOURCE})
  add_executable(${src} src/${src}.cpp)
  target_link_libraries(${src} ${FCL_LIBRARIES} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES})
endforeach(src)
