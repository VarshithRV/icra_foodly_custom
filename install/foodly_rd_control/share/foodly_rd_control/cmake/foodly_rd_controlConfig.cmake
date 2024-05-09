# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_foodly_rd_control_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED foodly_rd_control_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(foodly_rd_control_FOUND FALSE)
  elseif(NOT foodly_rd_control_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(foodly_rd_control_FOUND FALSE)
  endif()
  return()
endif()
set(_foodly_rd_control_CONFIG_INCLUDED TRUE)

# output package information
if(NOT foodly_rd_control_FIND_QUIETLY)
  message(STATUS "Found foodly_rd_control: 0.1.0 (${foodly_rd_control_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'foodly_rd_control' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${foodly_rd_control_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(foodly_rd_control_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${foodly_rd_control_DIR}/${_extra}")
endforeach()
