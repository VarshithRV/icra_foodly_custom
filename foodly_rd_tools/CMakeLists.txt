cmake_minimum_required(VERSION 3.8)
project(foodly_rd_tools)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  scripts
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
