cmake_minimum_required(VERSION 3.8)
project(usv_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_library(camera_component SHARED src/camera_node.cpp)
target_include_directories(camera_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(camera_component
  PRIVATE "USV_PERCEPTION_BUILDING_LIBRARY")
ament_target_dependencies(camera_component
  rclcpp rclcpp_components sensor_msgs cv_bridge image_transport OpenCV)
rclcpp_components_register_node(camera_component
  PLUGIN "usv_perception::CameraNode"
  EXECUTABLE camera_node)

add_library(processing_component SHARED src/processing_node.cpp)
target_include_directories(processing_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(processing_component
  PRIVATE "USV_PERCEPTION_BUILDING_LIBRARY")
ament_target_dependencies(processing_component
  rclcpp rclcpp_components sensor_msgs std_msgs cv_bridge image_transport OpenCV)
rclcpp_components_register_node(processing_component
  PLUGIN "usv_perception::ProcessingNode"
  EXECUTABLE processing_node)

install(TARGETS
  camera_component
  processing_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
