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
include esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/depend.make

# Include the progress variables for this target.
include esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/flags.make

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/gdbstub.c.obj: esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/flags.make
esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/gdbstub.c.obj: C:/Users/pawel/Desktop/esp-idf/components/esp_gdbstub/src/gdbstub.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/gdbstub.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_esp_gdbstub.dir\src\gdbstub.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\src\gdbstub.c

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/gdbstub.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_gdbstub.dir/src/gdbstub.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\src\gdbstub.c > CMakeFiles\__idf_esp_gdbstub.dir\src\gdbstub.c.i

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/gdbstub.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_gdbstub.dir/src/gdbstub.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\src\gdbstub.c -o CMakeFiles\__idf_esp_gdbstub.dir\src\gdbstub.c.s

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/packet.c.obj: esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/flags.make
esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/packet.c.obj: C:/Users/pawel/Desktop/esp-idf/components/esp_gdbstub/src/packet.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/packet.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_esp_gdbstub.dir\src\packet.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\src\packet.c

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/packet.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_gdbstub.dir/src/packet.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\src\packet.c > CMakeFiles\__idf_esp_gdbstub.dir\src\packet.c.i

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/packet.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_gdbstub.dir/src/packet.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\src\packet.c -o CMakeFiles\__idf_esp_gdbstub.dir\src\packet.c.s

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/esp32/gdbstub_esp32.c.obj: esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/flags.make
esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/esp32/gdbstub_esp32.c.obj: C:/Users/pawel/Desktop/esp-idf/components/esp_gdbstub/esp32/gdbstub_esp32.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/esp32/gdbstub_esp32.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_esp_gdbstub.dir\esp32\gdbstub_esp32.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\esp32\gdbstub_esp32.c

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/esp32/gdbstub_esp32.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_gdbstub.dir/esp32/gdbstub_esp32.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\esp32\gdbstub_esp32.c > CMakeFiles\__idf_esp_gdbstub.dir\esp32\gdbstub_esp32.c.i

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/esp32/gdbstub_esp32.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_gdbstub.dir/esp32/gdbstub_esp32.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\esp32\gdbstub_esp32.c -o CMakeFiles\__idf_esp_gdbstub.dir\esp32\gdbstub_esp32.c.s

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/xtensa/gdbstub_xtensa.c.obj: esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/flags.make
esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/xtensa/gdbstub_xtensa.c.obj: C:/Users/pawel/Desktop/esp-idf/components/esp_gdbstub/xtensa/gdbstub_xtensa.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/xtensa/gdbstub_xtensa.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_esp_gdbstub.dir\xtensa\gdbstub_xtensa.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\xtensa\gdbstub_xtensa.c

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/xtensa/gdbstub_xtensa.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_gdbstub.dir/xtensa/gdbstub_xtensa.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\xtensa\gdbstub_xtensa.c > CMakeFiles\__idf_esp_gdbstub.dir\xtensa\gdbstub_xtensa.c.i

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/xtensa/gdbstub_xtensa.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_gdbstub.dir/xtensa/gdbstub_xtensa.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub\xtensa\gdbstub_xtensa.c -o CMakeFiles\__idf_esp_gdbstub.dir\xtensa\gdbstub_xtensa.c.s

# Object files for target __idf_esp_gdbstub
__idf_esp_gdbstub_OBJECTS = \
"CMakeFiles/__idf_esp_gdbstub.dir/src/gdbstub.c.obj" \
"CMakeFiles/__idf_esp_gdbstub.dir/src/packet.c.obj" \
"CMakeFiles/__idf_esp_gdbstub.dir/esp32/gdbstub_esp32.c.obj" \
"CMakeFiles/__idf_esp_gdbstub.dir/xtensa/gdbstub_xtensa.c.obj"

# External object files for target __idf_esp_gdbstub
__idf_esp_gdbstub_EXTERNAL_OBJECTS =

esp-idf/esp_gdbstub/libesp_gdbstub.a: esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/gdbstub.c.obj
esp-idf/esp_gdbstub/libesp_gdbstub.a: esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/src/packet.c.obj
esp-idf/esp_gdbstub/libesp_gdbstub.a: esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/esp32/gdbstub_esp32.c.obj
esp-idf/esp_gdbstub/libesp_gdbstub.a: esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/xtensa/gdbstub_xtensa.c.obj
esp-idf/esp_gdbstub/libesp_gdbstub.a: esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/build.make
esp-idf/esp_gdbstub/libesp_gdbstub.a: esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libesp_gdbstub.a"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && $(CMAKE_COMMAND) -P CMakeFiles\__idf_esp_gdbstub.dir\cmake_clean_target.cmake
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\__idf_esp_gdbstub.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/build: esp-idf/esp_gdbstub/libesp_gdbstub.a

.PHONY : esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/build

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/clean:
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub && $(CMAKE_COMMAND) -P CMakeFiles\__idf_esp_gdbstub.dir\cmake_clean.cmake
.PHONY : esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/clean

esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\pawel\Desktop\Grant\Project\firmware C:\Users\pawel\Desktop\esp-idf\components\esp_gdbstub C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub C:\Users\pawel\Desktop\Grant\Project\firmware\CodeB\esp-idf\esp_gdbstub\CMakeFiles\__idf_esp_gdbstub.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/esp_gdbstub/CMakeFiles/__idf_esp_gdbstub.dir/depend

