cmake_minimum_required(VERSION 3.5)
project(obstacle_detector)

set(CMAKE_CXX_STANDARD 14)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry2 REQUIRED)
find_package(obstacle_detector_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost 1.54.0 REQUIRED system)

include_directories(include)

add_library(obstacle_detector_core SHARED
  src/obstacle_extractor.cpp
  src/obstacle_tracker.cpp
)

# target_compile_definitions(
#   PRIVATE "OBSTACLE_DETECTOR_BUILDING_LIBRARY"
# )

target_compile_options(obstacle_detector_core PUBLIC -Wall)
ament_target_dependencies(obstacle_detector_core
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  geometry2
  obstacle_detector_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  laser_geometry
)

target_include_directories(obstacle_detector_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

ament_export_targets(export_obstacle_detector_core HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  geometry2
  obstacle_detector_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  laser_geometry
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS obstacle_detector_core
  EXPORT export_obstacle_detector_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

add_executable(obstacle_extractor_node
  src/nodes/obstacle_extractor_node.cpp
  # src/obstacle_extractor.cpp
)

ament_target_dependencies(obstacle_extractor_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  geometry2
  obstacle_detector_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  laser_geometry
)

target_link_libraries(obstacle_extractor_node
  obstacle_detector_core
  ${ARMADILLO_LIBRARIES}
)

add_executable(obstacle_tracker_node
  src/nodes/obstacle_tracker_node.cpp
  # src/obstacle_tracker.cpp
)

ament_target_dependencies(obstacle_tracker_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "geometry2"
  "obstacle_detector_interfaces"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "laser_geometry"
)

target_link_libraries(obstacle_tracker_node
  obstacle_detector_core
  ${ARMADILLO_LIBRARIES}
)

install(TARGETS obstacle_extractor_node obstacle_tracker_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

#
# Install launch files and params files
#
install(DIRECTORY
  launch
  params
  resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
