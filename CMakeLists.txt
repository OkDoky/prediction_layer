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

generate_dynamic_reconfigure_options(cfg/PredictionLayer.cfg)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES prediction_layer
 CATKIN_DEPENDS costmap_2d dynamic_reconfigure geometry_msgs pluginlib roscpp sensor_msgs tf2_ros obstacle_detector
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/prediction_layer.cpp)
add_dependencies(${PROJECT_NAME} ${prediction_layer_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(FILES prediction_layer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RENTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES prediction_layer_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)