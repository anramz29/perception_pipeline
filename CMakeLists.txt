cmake_minimum_required(VERSION 3.8)
project(perception_pipeline)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ── Python (CV / model inference) ────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS perception_pipeline/detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detector_node
)

install(PROGRAMS perception_pipeline/pose_estimation_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_estimation_node
)

# ── Shared resources ─────────────────────────────────────────────────────────
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
