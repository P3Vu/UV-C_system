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

# Include any dependencies generated for this target.
include esp-idf/heap/CMakeFiles/__idf_heap.dir/depend.make

# Include the progress variables for this target.
include esp-idf/heap/CMakeFiles/__idf_heap.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/heap/CMakeFiles/__idf_heap.dir/flags.make

esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps.c.obj: esp-idf/heap/CMakeFiles/__idf_heap.dir/flags.make
esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps.c.obj: C:/Users/pawel/Desktop/esp-idf/components/heap/heap_caps.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_heap.dir\heap_caps.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\heap\heap_caps.c

esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_heap.dir/heap_caps.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\heap\heap_caps.c > CMakeFiles\__idf_heap.dir\heap_caps.c.i

esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_heap.dir/heap_caps.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\heap\heap_caps.c -o CMakeFiles\__idf_heap.dir\heap_caps.c.s

esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps_init.c.obj: esp-idf/heap/CMakeFiles/__idf_heap.dir/flags.make
esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps_init.c.obj: C:/Users/pawel/Desktop/esp-idf/components/heap/heap_caps_init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps_init.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_heap.dir\heap_caps_init.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\heap\heap_caps_init.c

esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps_init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_heap.dir/heap_caps_init.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\heap\heap_caps_init.c > CMakeFiles\__idf_heap.dir\heap_caps_init.c.i

esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps_init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_heap.dir/heap_caps_init.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\heap\heap_caps_init.c -o CMakeFiles\__idf_heap.dir\heap_caps_init.c.s

esp-idf/heap/CMakeFiles/__idf_heap.dir/multi_heap.c.obj: esp-idf/heap/CMakeFiles/__idf_heap.dir/flags.make
esp-idf/heap/CMakeFiles/__idf_heap.dir/multi_heap.c.obj: C:/Users/pawel/Desktop/esp-idf/components/heap/multi_heap.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/heap/CMakeFiles/__idf_heap.dir/multi_heap.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_heap.dir\multi_heap.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\heap\multi_heap.c

esp-idf/heap/CMakeFiles/__idf_heap.dir/multi_heap.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_heap.dir/multi_heap.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\heap\multi_heap.c > CMakeFiles\__idf_heap.dir\multi_heap.c.i

esp-idf/heap/CMakeFiles/__idf_heap.dir/multi_heap.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_heap.dir/multi_heap.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\heap\multi_heap.c -o CMakeFiles\__idf_heap.dir\multi_heap.c.s

# Object files for target __idf_heap
__idf_heap_OBJECTS = \
"CMakeFiles/__idf_heap.dir/heap_caps.c.obj" \
"CMakeFiles/__idf_heap.dir/heap_caps_init.c.obj" \
"CMakeFiles/__idf_heap.dir/multi_heap.c.obj"

# External object files for target __idf_heap
__idf_heap_EXTERNAL_OBJECTS =

esp-idf/heap/libheap.a: esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps.c.obj
esp-idf/heap/libheap.a: esp-idf/heap/CMakeFiles/__idf_heap.dir/heap_caps_init.c.obj
esp-idf/heap/libheap.a: esp-idf/heap/CMakeFiles/__idf_heap.dir/multi_heap.c.obj
esp-idf/heap/libheap.a: esp-idf/heap/CMakeFiles/__idf_heap.dir/build.make
esp-idf/heap/libheap.a: esp-idf/heap/CMakeFiles/__idf_heap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libheap.a"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && $(CMAKE_COMMAND) -P CMakeFiles\__idf_heap.dir\cmake_clean_target.cmake
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\__idf_heap.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/heap/CMakeFiles/__idf_heap.dir/build: esp-idf/heap/libheap.a

.PHONY : esp-idf/heap/CMakeFiles/__idf_heap.dir/build

esp-idf/heap/CMakeFiles/__idf_heap.dir/clean:
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap && $(CMAKE_COMMAND) -P CMakeFiles\__idf_heap.dir\cmake_clean.cmake
.PHONY : esp-idf/heap/CMakeFiles/__idf_heap.dir/clean

esp-idf/heap/CMakeFiles/__idf_heap.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\pawel\Desktop\Grant\Project\firmware C:\Users\pawel\Desktop\esp-idf\components\heap C:\Users\pawel\Desktop\Grant\Project\firmware\CMake C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\heap\CMakeFiles\__idf_heap.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/heap/CMakeFiles/__idf_heap.dir/depend

