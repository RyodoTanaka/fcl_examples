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

##################
## Original Lib ##
##################
add_library(FCLExampleUtils SHARED
  src/broadphase/utils.cpp
  src/broadphase/utils.h
  )

set(ORIGINAL_LIBS
  FCLExampleUtils
  )

###########
## Build ##
###########
set(NARROW_PHASE_SOURCE
  np_sphere_sphere
  np_box_sphere
  np_box_box
  np_cylinder_sphere
  np_cylinder_box
  np_cylinder_cylinder
  )

set(BROAD_PHASE_SOURCE
  bp_dynamic_aabb_tree
  )

foreach(src ${NARROW_PHASE_SOURCE})
  add_executable(${src} src/narrowphase/${src}.cpp)
  target_link_libraries(${src} ${FCL_LIBRARIES} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES})
endforeach(src)

foreach(src ${BROAD_PHASE_SOURCE})
  add_executable(${src} src/broadphase/${src}.cpp)
  target_link_libraries(${src} ${ORIGINAL_LIBS} ${FCL_LIBRARIES} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES})
endforeach(src)
