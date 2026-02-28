cmake_minimum_required(VERSION 3.8)
project(onboard_detector)

# --------------------------------------------------
# Compile options
# --------------------------------------------------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

# Set RPATH so binary can find libraries
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# --------------------------------------------------
# Find dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# For services
find_package(rosidl_default_generators REQUIRED)

# --------------------------------------------------
# Service definition
# --------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDynamicObstacles.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# --------------------------------------------------
# Include directories
# --------------------------------------------------
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# --------------------------------------------------
# Library
# --------------------------------------------------
add_library(${PROJECT_NAME}_lib
  include/${PROJECT_NAME}/dynamicDetector.cpp
  include/${PROJECT_NAME}/uvDetector.cpp
  include/${PROJECT_NAME}/dbscan.cpp
  include/${PROJECT_NAME}/kalmanFilter.cpp
  include/${PROJECT_NAME}/lidarDetector.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  image_transport
  vision_msgs
  pcl_conversions
  tf2
  tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${PROJECT_NAME}_lib
  "${cpp_typesupport_target}")

# --------------------------------------------------
# Executable
# --------------------------------------------------
add_executable(detector_node src/detector_node.cpp)

target_link_libraries(detector_node ${PROJECT_NAME}_lib)
ament_target_dependencies(detector_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  image_transport
  vision_msgs
  pcl_conversions
  tf2
  tf2_geometry_msgs
)

# --------------------------------------------------
# --------------------------------------------------
# Install
# --------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}_lib
  detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install Python node entrypoints as executables
install(PROGRAMS
  scripts/yolo_detector/yolo_detector_node.py
  scripts/yolo_detector/yolov11_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all scripts (for weights, modules, etc.)
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# --------------------------------------------------
# Export
# --------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_package()