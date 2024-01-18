cmake_minimum_required(VERSION 3.8)
project(tiago_dual_moveit_config)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
  pal_register_modules(module/move_group.yaml)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the moveit setup assistant to avoid some warnings when launching move_group
install(FILES
    .setup_assistant
    DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch config)

