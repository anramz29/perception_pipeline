cmake_minimum_required(VERSION 3.8)
project(perception_pipeline)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ── ROS dependencies ──────────────────────────────────────────────────────────
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ── ONNX Runtime ──────────────────────────────────────────────────────────────
find_package(onnxruntime QUIET)
if(NOT onnxruntime_FOUND)
  find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS /usr/include/onnxruntime /usr/local/include/onnxruntime /opt/onnxruntime/include)
  find_library(ONNXRUNTIME_LIB
    NAMES onnxruntime
    PATHS /usr/lib /usr/local/lib /opt/onnxruntime/lib)
  if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
    add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
      IMPORTED_LOCATION "${ONNXRUNTIME_LIB}"
      INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR
      "ONNX Runtime C++ not found.\n"
      "Install from https://github.com/microsoft/onnxruntime/releases\n"
      "or: sudo apt install libonnxruntime-dev (if available for your distro)")
  endif()
endif()

# ── C++ detector node ─────────────────────────────────────────────────────────
add_executable(detector_node src/detector_node.cpp)
target_compile_features(detector_node PUBLIC cxx_std_17)
ament_target_dependencies(detector_node
  rclcpp sensor_msgs vision_msgs cv_bridge OpenCV)
target_link_libraries(detector_node onnxruntime::onnxruntime)

install(TARGETS detector_node
  DESTINATION lib/${PROJECT_NAME})

# ── Python (pose estimation — requires FoundationPose/PyTorch) ────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS perception_pipeline/pose_estimation_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_estimation_node)

# ── Shared resources ──────────────────────────────────────────────────────────
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
