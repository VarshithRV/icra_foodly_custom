# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/barracuda/icra_ws/src/icra_foodly_custom/foodly_rd_examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/barracuda/icra_ws/src/icra_foodly_custom/build/foodly_rd_examples

# Include any dependencies generated for this target.
include CMakeFiles/pose_presets.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/pose_presets.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/pose_presets.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pose_presets.dir/flags.make

CMakeFiles/pose_presets.dir/src/pose_presets.cpp.o: CMakeFiles/pose_presets.dir/flags.make
CMakeFiles/pose_presets.dir/src/pose_presets.cpp.o: /home/barracuda/icra_ws/src/icra_foodly_custom/foodly_rd_examples/src/pose_presets.cpp
CMakeFiles/pose_presets.dir/src/pose_presets.cpp.o: CMakeFiles/pose_presets.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/barracuda/icra_ws/src/icra_foodly_custom/build/foodly_rd_examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pose_presets.dir/src/pose_presets.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pose_presets.dir/src/pose_presets.cpp.o -MF CMakeFiles/pose_presets.dir/src/pose_presets.cpp.o.d -o CMakeFiles/pose_presets.dir/src/pose_presets.cpp.o -c /home/barracuda/icra_ws/src/icra_foodly_custom/foodly_rd_examples/src/pose_presets.cpp

CMakeFiles/pose_presets.dir/src/pose_presets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pose_presets.dir/src/pose_presets.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/barracuda/icra_ws/src/icra_foodly_custom/foodly_rd_examples/src/pose_presets.cpp > CMakeFiles/pose_presets.dir/src/pose_presets.cpp.i

CMakeFiles/pose_presets.dir/src/pose_presets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pose_presets.dir/src/pose_presets.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/barracuda/icra_ws/src/icra_foodly_custom/foodly_rd_examples/src/pose_presets.cpp -o CMakeFiles/pose_presets.dir/src/pose_presets.cpp.s

# Object files for target pose_presets
pose_presets_OBJECTS = \
"CMakeFiles/pose_presets.dir/src/pose_presets.cpp.o"

# External object files for target pose_presets
pose_presets_EXTERNAL_OBJECTS =

libpose_presets.so: CMakeFiles/pose_presets.dir/src/pose_presets.cpp.o
libpose_presets.so: CMakeFiles/pose_presets.dir/build.make
libpose_presets.so: /usr/lib/x86_64-linux-gnu/liborocos-kdl.so
libpose_presets.so: /opt/ros/humble/lib/libtf2_ros.so
libpose_presets.so: /opt/ros/humble/lib/libtf2.so
libpose_presets.so: /opt/ros/humble/lib/libmessage_filters.so
libpose_presets.so: /opt/ros/humble/lib/librclcpp_action.so
libpose_presets.so: /opt/ros/humble/lib/librclcpp.so
libpose_presets.so: /opt/ros/humble/lib/liblibstatistics_collector.so
libpose_presets.so: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
libpose_presets.so: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
libpose_presets.so: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
libpose_presets.so: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
libpose_presets.so: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
libpose_presets.so: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
libpose_presets.so: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
libpose_presets.so: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
libpose_presets.so: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
libpose_presets.so: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
libpose_presets.so: /opt/ros/humble/lib/librcl_action.so
libpose_presets.so: /opt/ros/humble/lib/librcl.so
libpose_presets.so: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
libpose_presets.so: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
libpose_presets.so: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
libpose_presets.so: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
libpose_presets.so: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
libpose_presets.so: /opt/ros/humble/lib/librcl_yaml_param_parser.so
libpose_presets.so: /opt/ros/humble/lib/libyaml.so
libpose_presets.so: /opt/ros/humble/lib/libtracetools.so
libpose_presets.so: /opt/ros/humble/lib/librmw_implementation.so
libpose_presets.so: /opt/ros/humble/lib/libament_index_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librcl_logging_spdlog.so
libpose_presets.so: /opt/ros/humble/lib/librcl_logging_interface.so
libpose_presets.so: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_fastrtps_c.so
libpose_presets.so: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
libpose_presets.so: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
libpose_presets.so: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_fastrtps_c.so
libpose_presets.so: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
libpose_presets.so: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_c.so
libpose_presets.so: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
libpose_presets.so: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_introspection_c.so
libpose_presets.so: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
libpose_presets.so: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
libpose_presets.so: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
libpose_presets.so: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
libpose_presets.so: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
libpose_presets.so: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_fastrtps_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_fastrtps_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libfastcdr.so.1.0.24
libpose_presets.so: /opt/ros/humble/lib/librmw.so
libpose_presets.so: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_introspection_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
libpose_presets.so: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
libpose_presets.so: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
libpose_presets.so: /opt/ros/humble/lib/libtf2_msgs__rosidl_generator_py.so
libpose_presets.so: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
libpose_presets.so: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
libpose_presets.so: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_c.so
libpose_presets.so: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
libpose_presets.so: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
libpose_presets.so: /opt/ros/humble/lib/libtf2_msgs__rosidl_generator_c.so
libpose_presets.so: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
libpose_presets.so: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
libpose_presets.so: /opt/ros/humble/lib/libaction_msgs__rosidl_generator_py.so
libpose_presets.so: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
libpose_presets.so: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_c.so
libpose_presets.so: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
libpose_presets.so: /opt/ros/humble/lib/libaction_msgs__rosidl_generator_c.so
libpose_presets.so: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
libpose_presets.so: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_generator_py.so
libpose_presets.so: /usr/lib/x86_64-linux-gnu/libpython3.10.so
libpose_presets.so: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
libpose_presets.so: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_generator_c.so
libpose_presets.so: /opt/ros/humble/lib/librosidl_typesupport_c.so
libpose_presets.so: /opt/ros/humble/lib/librosidl_runtime_c.so
libpose_presets.so: /opt/ros/humble/lib/librcpputils.so
libpose_presets.so: /opt/ros/humble/lib/librcutils.so
libpose_presets.so: CMakeFiles/pose_presets.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/barracuda/icra_ws/src/icra_foodly_custom/build/foodly_rd_examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libpose_presets.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pose_presets.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pose_presets.dir/build: libpose_presets.so
.PHONY : CMakeFiles/pose_presets.dir/build

CMakeFiles/pose_presets.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pose_presets.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pose_presets.dir/clean

CMakeFiles/pose_presets.dir/depend:
	cd /home/barracuda/icra_ws/src/icra_foodly_custom/build/foodly_rd_examples && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/barracuda/icra_ws/src/icra_foodly_custom/foodly_rd_examples /home/barracuda/icra_ws/src/icra_foodly_custom/foodly_rd_examples /home/barracuda/icra_ws/src/icra_foodly_custom/build/foodly_rd_examples /home/barracuda/icra_ws/src/icra_foodly_custom/build/foodly_rd_examples /home/barracuda/icra_ws/src/icra_foodly_custom/build/foodly_rd_examples/CMakeFiles/pose_presets.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pose_presets.dir/depend
