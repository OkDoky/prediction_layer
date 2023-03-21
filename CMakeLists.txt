cmake_minimum_required(VERSION 3.0.2)
project(prediction_layer)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  pluginlib
  roscpp
  # rxros
  sensor_msgs
  tf2_ros
  tf2
  obstacle_detector
  std_srvs
  tf2_geometry_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)
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

add_message_files(
  FILES
  PolygonBoundary.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS 
  include
 LIBRARIES 
  prediction_layer
 CATKIN_DEPENDS 
  costmap_2d 
  dynamic_reconfigure 
  geometry_msgs 
  pluginlib 
  roscpp 
  sensor_msgs 
  tf2_ros 
  obstacle_detector
  tf2
  message_filters
  message_generation
  nav_msgs
  std_srvs
  tf2_geometry_msgs
 DEPENDS
  Boost 
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
)

add_library(
  prediction_layer 
  src/prediction_layer.cpp
  src/obstacles_buffer.cpp
)

#
# Build rviz plugins
#
# add_library(
#   ${PROJECT_NAME}_display
#   src/display/boundary_visual.cpp include/${PROJECT_NAME}/display/boundary_visual.h
# )
# target_link_libraries(${PROJECT_NAME}_display ${catkin_LIBRARIES})
# add_dependencies(${PROJECT_NAME}_display ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  prediction_layer 
  ${catkin_LIBRARIES}
)

add_dependencies(
  prediction_layer 
  ${prediction_layer_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

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
