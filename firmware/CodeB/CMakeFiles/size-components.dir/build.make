# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = C:\Users\pawel\.espressif\tools\cmake\3.13.4\bin\cmake.exe

# The command to remove a file.
RM = C:\Users\pawel\.espressif\tools\cmake\3.13.4\bin\cmake.exe -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\pawel\Desktop\Grant\Project\firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\pawel\Desktop\Grant\Project\firmware\CMake

# Utility rule file for size-components.

# Include the progress variables for this target.
include CMakeFiles/size-components.dir/progress.make

CMakeFiles/size-components: UV.elf
	python C:/Users/pawel/Desktop/esp-idf/tools/idf_size.py --archives C:/Users/pawel/Desktop/Grant/Project/firmware/CMake/UV.map

size-components: CMakeFiles/size-components
size-components: CMakeFiles/size-components.dir/build.make

.PHONY : size-components

# Rule to build all files generated by this target.
CMakeFiles/size-components.dir/build: size-components

.PHONY : CMakeFiles/size-components.dir/build

CMakeFiles/size-components.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\size-components.dir\cmake_clean.cmake
.PHONY : CMakeFiles/size-components.dir/clean

CMakeFiles/size-components.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\pawel\Desktop\Grant\Project\firmware C:\Users\pawel\Desktop\Grant\Project\firmware C:\Users\pawel\Desktop\Grant\Project\firmware\CMake C:\Users\pawel\Desktop\Grant\Project\firmware\CMake C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles\size-components.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/size-components.dir/depend
