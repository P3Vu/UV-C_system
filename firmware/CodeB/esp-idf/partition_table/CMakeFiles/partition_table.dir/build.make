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
CMAKE_BINARY_DIR = C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB

# Utility rule file for partition_table.

# Include the progress variables for this target.
include esp-idf/partition_table/CMakeFiles/partition_table.dir/progress.make

esp-idf/partition_table/CMakeFiles/partition_table: partition_table/partition-table.bin
esp-idf/partition_table/CMakeFiles/partition_table: partition_table/partition-table.bin
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\partition_table && C:\Users\pawel\.espressif\tools\cmake\3.13.4\bin\cmake.exe -E echo "Partition table binary generated. Contents:"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\partition_table && C:\Users\pawel\.espressif\tools\cmake\3.13.4\bin\cmake.exe -E echo *******************************************************************************
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\partition_table && python C:/Users/pawel/Desktop/esp-idf/components/partition_table/gen_esp32part.py -q --offset 0x8000 --flash-size 2MB C:/Users/pawel/Desktop/Grant/Project/firmware/CodeB/partition_table/partition-table.bin
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\partition_table && C:\Users\pawel\.espressif\tools\cmake\3.13.4\bin\cmake.exe -E echo *******************************************************************************

partition_table/partition-table.bin: C:/Users/pawel/Desktop/esp-idf/components/partition_table/partitions_singleapp.csv
partition_table/partition-table.bin: C:/Users/pawel/Desktop/esp-idf/components/partition_table/gen_esp32part.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ../../partition_table/partition-table.bin"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\partition_table && python C:/Users/pawel/Desktop/esp-idf/components/partition_table/gen_esp32part.py -q --offset 0x8000 --flash-size 2MB C:/Users/pawel/Desktop/esp-idf/components/partition_table/partitions_singleapp.csv C:/Users/pawel/Desktop/Grant/Project/firmware/CodeB/partition_table/partition-table.bin

partition_table: esp-idf/partition_table/CMakeFiles/partition_table
partition_table: partition_table/partition-table.bin
partition_table: esp-idf/partition_table/CMakeFiles/partition_table.dir/build.make

.PHONY : partition_table

# Rule to build all files generated by this target.
esp-idf/partition_table/CMakeFiles/partition_table.dir/build: partition_table

.PHONY : esp-idf/partition_table/CMakeFiles/partition_table.dir/build

esp-idf/partition_table/CMakeFiles/partition_table.dir/clean:
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\partition_table && $(CMAKE_COMMAND) -P CMakeFiles\partition_table.dir\cmake_clean.cmake
.PHONY : esp-idf/partition_table/CMakeFiles/partition_table.dir/clean

esp-idf/partition_table/CMakeFiles/partition_table.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\pawel\Desktop\Grant\Project\firmware C:\Users\pawel\Desktop\esp-idf\components\partition_table C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\partition_table C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\partition_table\CMakeFiles\partition_table.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/partition_table/CMakeFiles/partition_table.dir/depend

