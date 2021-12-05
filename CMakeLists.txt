cmake_minimum_required(VERSION 3.5)
project(obstacle_detector)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(rviz REQUIRED)
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

add_executable(obstacle_extractor_node
  src/nodes/obstacle_extractor_node.cpp
  src/obstacle_extractor.cpp
)

ament_target_dependencies(obstacle_extractor_node
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

target_link_libraries(obstacle_extractor_node
  ${ARMADILLO_LIBRARIES}
)

add_executable(obstacle_tracker_node
  src/nodes/obstacle_tracker_node.cpp
  src/obstacle_tracker.cpp
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
  ${ARMADILLO_LIBRARIES}
)

install(TARGETS obstacle_extractor_node obstacle_tracker_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  # LIBRARY DESTINATION lib/${PROJECT_NAME}
  # ARCHIVE DESTINATION lib
)

# add_executable(obstacle_tracker_node src/nodes/obstacle_tracker_node.cpp)
# target_link_libraries(obstacle_tracker_node obstacle_tracker)

#
# Install launch files
#
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
