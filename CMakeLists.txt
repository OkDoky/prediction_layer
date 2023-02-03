cmake_minimum_required(VERSION 3.0.2)
project(prediction_layer)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  pluginlib
  roscpp
  # rxros
  sensor_msgs
  tf2_ros
  obstacle_detector
)

## Enable C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORT_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORT_CXX0X)
if(COMPILER_SUPPORT_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORT_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0X")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/PredictionLayer.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES prediction_layer
 CATKIN_DEPENDS costmap_2d dynamic_reconfigure geometry_msgs pluginlib roscpp sensor_msgs tf2_ros obstacle_detector
 DEPENDS 
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_library(
  prediction_layer 
  src/prediction_layer.cpp
  src/circle_to_circumscribe_polygon.cpp
)
target_link_libraries(
  prediction_layer 
  ${catkin_LIBRARIES}
)

add_dependencies(
  prediction_layer 
  ${prediction_layer_EXPORTED_TARGETS}
)

# add_executable(
#   prediction_layer_node
#   src/prediction_layer.cpp
#   # src/circle_to_circumscribe_polygon.cpp
# )

install(TARGETS prediction_layer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES prediction_layer_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)