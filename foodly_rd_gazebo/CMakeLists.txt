cmake_minimum_required(VERSION 3.5)
project(foodly_rd_gazebo)

find_package(ament_cmake REQUIRED COMPONENTS)

install(DIRECTORY
  config
  launch
  worlds
  gui
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright checks
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
