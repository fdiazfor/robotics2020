# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/clion/129/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/129/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca

# Utility rule file for MyFirstComp_autogen.

# Include the progress variables for this target.
include src/CMakeFiles/MyFirstComp_autogen.dir/progress.make

src/CMakeFiles/MyFirstComp_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target MyFirstComp"
	cd /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca/src && /snap/clion/129/bin/cmake/linux/bin/cmake -E cmake_autogen /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca/src/CMakeFiles/MyFirstComp_autogen.dir/AutogenInfo.json Debug

MyFirstComp_autogen: src/CMakeFiles/MyFirstComp_autogen
MyFirstComp_autogen: src/CMakeFiles/MyFirstComp_autogen.dir/build.make

.PHONY : MyFirstComp_autogen

# Rule to build all files generated by this target.
src/CMakeFiles/MyFirstComp_autogen.dir/build: MyFirstComp_autogen

.PHONY : src/CMakeFiles/MyFirstComp_autogen.dir/build

src/CMakeFiles/MyFirstComp_autogen.dir/clean:
	cd /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca/src && $(CMAKE_COMMAND) -P CMakeFiles/MyFirstComp_autogen.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/MyFirstComp_autogen.dir/clean

src/CMakeFiles/MyFirstComp_autogen.dir/depend:
	cd /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca/src /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca/src /home/salabeta/robocomp/components/fdiazfor/Robotica/practica2/chocachoca/src/CMakeFiles/MyFirstComp_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/MyFirstComp_autogen.dir/depend

