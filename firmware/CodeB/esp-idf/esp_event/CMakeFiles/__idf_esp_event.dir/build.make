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

# Include any dependencies generated for this target.
include esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/depend.make

# Include the progress variables for this target.
include esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/flags.make

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/default_event_loop.c.obj: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/flags.make
esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/default_event_loop.c.obj: C:/Users/pawel/Desktop/esp-idf/components/esp_event/default_event_loop.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/default_event_loop.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_esp_event.dir\default_event_loop.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\esp_event\default_event_loop.c

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/default_event_loop.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_event.dir/default_event_loop.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\esp_event\default_event_loop.c > CMakeFiles\__idf_esp_event.dir\default_event_loop.c.i

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/default_event_loop.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_event.dir/default_event_loop.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\esp_event\default_event_loop.c -o CMakeFiles\__idf_esp_event.dir\default_event_loop.c.s

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event.c.obj: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/flags.make
esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event.c.obj: C:/Users/pawel/Desktop/esp-idf/components/esp_event/esp_event.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_esp_event.dir\esp_event.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\esp_event\esp_event.c

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_event.dir/esp_event.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\esp_event\esp_event.c > CMakeFiles\__idf_esp_event.dir\esp_event.c.i

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_event.dir/esp_event.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\esp_event\esp_event.c -o CMakeFiles\__idf_esp_event.dir\esp_event.c.s

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event_private.c.obj: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/flags.make
esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event_private.c.obj: C:/Users/pawel/Desktop/esp-idf/components/esp_event/esp_event_private.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event_private.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_esp_event.dir\esp_event_private.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\esp_event\esp_event_private.c

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event_private.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_event.dir/esp_event_private.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\esp_event\esp_event_private.c > CMakeFiles\__idf_esp_event.dir\esp_event_private.c.i

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event_private.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_event.dir/esp_event_private.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\esp_event\esp_event_private.c -o CMakeFiles\__idf_esp_event.dir\esp_event_private.c.s

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_loop_legacy.c.obj: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/flags.make
esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_loop_legacy.c.obj: C:/Users/pawel/Desktop/esp-idf/components/esp_event/event_loop_legacy.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_loop_legacy.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_esp_event.dir\event_loop_legacy.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\esp_event\event_loop_legacy.c

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_loop_legacy.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_event.dir/event_loop_legacy.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\esp_event\event_loop_legacy.c > CMakeFiles\__idf_esp_event.dir\event_loop_legacy.c.i

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_loop_legacy.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_event.dir/event_loop_legacy.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\esp_event\event_loop_legacy.c -o CMakeFiles\__idf_esp_event.dir\event_loop_legacy.c.s

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_send.c.obj: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/flags.make
esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_send.c.obj: C:/Users/pawel/Desktop/esp-idf/components/esp_event/event_send.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_send.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_esp_event.dir\event_send.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\esp_event\event_send.c

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_send.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_event.dir/event_send.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\esp_event\event_send.c > CMakeFiles\__idf_esp_event.dir\event_send.c.i

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_send.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_event.dir/event_send.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\esp_event\event_send.c -o CMakeFiles\__idf_esp_event.dir\event_send.c.s

# Object files for target __idf_esp_event
__idf_esp_event_OBJECTS = \
"CMakeFiles/__idf_esp_event.dir/default_event_loop.c.obj" \
"CMakeFiles/__idf_esp_event.dir/esp_event.c.obj" \
"CMakeFiles/__idf_esp_event.dir/esp_event_private.c.obj" \
"CMakeFiles/__idf_esp_event.dir/event_loop_legacy.c.obj" \
"CMakeFiles/__idf_esp_event.dir/event_send.c.obj"

# External object files for target __idf_esp_event
__idf_esp_event_EXTERNAL_OBJECTS =

esp-idf/esp_event/libesp_event.a: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/default_event_loop.c.obj
esp-idf/esp_event/libesp_event.a: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event.c.obj
esp-idf/esp_event/libesp_event.a: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/esp_event_private.c.obj
esp-idf/esp_event/libesp_event.a: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_loop_legacy.c.obj
esp-idf/esp_event/libesp_event.a: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/event_send.c.obj
esp-idf/esp_event/libesp_event.a: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/build.make
esp-idf/esp_event/libesp_event.a: esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library libesp_event.a"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && $(CMAKE_COMMAND) -P CMakeFiles\__idf_esp_event.dir\cmake_clean_target.cmake
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\__idf_esp_event.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/build: esp-idf/esp_event/libesp_event.a

.PHONY : esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/build

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/clean:
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event && $(CMAKE_COMMAND) -P CMakeFiles\__idf_esp_event.dir\cmake_clean.cmake
.PHONY : esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/clean

esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\pawel\Desktop\Grant\Project\firmware C:\Users\pawel\Desktop\esp-idf\components\esp_event C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_event\CMakeFiles\__idf_esp_event.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/esp_event/CMakeFiles/__idf_esp_event.dir/depend

