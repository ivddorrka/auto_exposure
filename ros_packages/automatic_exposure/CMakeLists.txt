cmake_minimum_required(VERSION 3.1.2)
project(automatic_exposure)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  std_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  image_geometry
  mrs_lib
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ExposureControl
  CATKIN_DEPENDS roscpp mrs_lib image_transport std_msgs
)

include_directories(include
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



## Declare a C++ library
add_library(ExposureControl src/ExposureControl.cpp)

## Add configure headers for dynamic reconfigure
add_dependencies(ExposureControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(ExposureControl 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}


  ) 
