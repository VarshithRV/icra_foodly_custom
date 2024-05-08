cmake_minimum_required(VERSION 3.5)
project(foodly_rd_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rt_manipulators_cpp REQUIRED)

set(LIBRARY_NAME "foodly_rd_hardware")
add_library(
  ${LIBRARY_NAME}
  SHARED
  src/foodly_rd_hardware.cpp
)
target_include_directories(
  ${LIBRARY_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${LIBRARY_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rt_manipulators_cpp
)

pluginlib_export_plugin_description_file(hardware_interface foodly_rd_hardware.xml)

install(
  TARGETS ${LIBRARY_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright checks
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${LIBRARY_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rt_manipulators_cpp
)

ament_package()
