cmake_minimum_required(VERSION 3.8)
project(perception_pipeline)

# ── C++17 MUST be set before anything else ──
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)

# ── ONNX Runtime ──
find_package(onnxruntime QUIET)
if(NOT onnxruntime_FOUND)
  find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS /usr/local/include /usr/include /opt/onnxruntime/include)
  find_library(ONNXRUNTIME_LIB
    NAMES onnxruntime
    PATHS /usr/local/lib /usr/lib /opt/onnxruntime/lib)
  if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
    add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
      IMPORTED_LOCATION "${ONNXRUNTIME_LIB}"
      INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "ONNX Runtime not found")
  endif()
endif()

# ── detector node ──
add_executable(detector_node src/detector_node.cpp)
target_compile_features(detector_node PUBLIC cxx_std_17)
ament_target_dependencies(detector_node
  rclcpp std_msgs sensor_msgs vision_msgs cv_bridge OpenCV)
target_link_libraries(detector_node onnxruntime::onnxruntime)

# ── bbox visualization node ──
add_executable(bbox_viz_node src/bbox_viz_node.cpp)
target_compile_features(bbox_viz_node PUBLIC cxx_std_17)
ament_target_dependencies(bbox_viz_node
  rclcpp sensor_msgs vision_msgs cv_bridge OpenCV message_filters)

  # -- 3D point localization Node
add_executable(point_localization_node src/point_localization_node.cpp)
target_compile_features(point_localization_node PUBLIC cxx_std_17)
ament_target_dependencies(point_localization_node
  rclcpp sensor_msgs vision_msgs cv_bridge geometry_msgs OpenCV message_filters)

install(TARGETS detector_node bbox_viz_node point_localization_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()