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
include esp-idf/efuse/CMakeFiles/__idf_efuse.dir/depend.make

# Include the progress variables for this target.
include esp-idf/efuse/CMakeFiles/__idf_efuse.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/efuse/CMakeFiles/__idf_efuse.dir/flags.make

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/esp32/esp_efuse_table.c.obj: esp-idf/efuse/CMakeFiles/__idf_efuse.dir/flags.make
esp-idf/efuse/CMakeFiles/__idf_efuse.dir/esp32/esp_efuse_table.c.obj: C:/Users/pawel/Desktop/esp-idf/components/efuse/esp32/esp_efuse_table.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/esp32/esp_efuse_table.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_efuse.dir\esp32\esp_efuse_table.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\efuse\esp32\esp_efuse_table.c

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/esp32/esp_efuse_table.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_efuse.dir/esp32/esp_efuse_table.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\efuse\esp32\esp_efuse_table.c > CMakeFiles\__idf_efuse.dir\esp32\esp_efuse_table.c.i

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/esp32/esp_efuse_table.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_efuse.dir/esp32/esp_efuse_table.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\efuse\esp32\esp_efuse_table.c -o CMakeFiles\__idf_efuse.dir\esp32\esp_efuse_table.c.s

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_api.c.obj: esp-idf/efuse/CMakeFiles/__idf_efuse.dir/flags.make
esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_api.c.obj: C:/Users/pawel/Desktop/esp-idf/components/efuse/src/esp_efuse_api.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_api.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_efuse.dir\src\esp_efuse_api.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\efuse\src\esp_efuse_api.c

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_api.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_efuse.dir/src/esp_efuse_api.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\efuse\src\esp_efuse_api.c > CMakeFiles\__idf_efuse.dir\src\esp_efuse_api.c.i

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_api.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_efuse.dir/src/esp_efuse_api.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\efuse\src\esp_efuse_api.c -o CMakeFiles\__idf_efuse.dir\src\esp_efuse_api.c.s

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_fields.c.obj: esp-idf/efuse/CMakeFiles/__idf_efuse.dir/flags.make
esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_fields.c.obj: C:/Users/pawel/Desktop/esp-idf/components/efuse/src/esp_efuse_fields.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_fields.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_efuse.dir\src\esp_efuse_fields.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\efuse\src\esp_efuse_fields.c

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_fields.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_efuse.dir/src/esp_efuse_fields.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\efuse\src\esp_efuse_fields.c > CMakeFiles\__idf_efuse.dir\src\esp_efuse_fields.c.i

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_fields.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_efuse.dir/src/esp_efuse_fields.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\efuse\src\esp_efuse_fields.c -o CMakeFiles\__idf_efuse.dir\src\esp_efuse_fields.c.s

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_utility.c.obj: esp-idf/efuse/CMakeFiles/__idf_efuse.dir/flags.make
esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_utility.c.obj: C:/Users/pawel/Desktop/esp-idf/components/efuse/src/esp_efuse_utility.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_utility.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_efuse.dir\src\esp_efuse_utility.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\efuse\src\esp_efuse_utility.c

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_utility.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_efuse.dir/src/esp_efuse_utility.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\efuse\src\esp_efuse_utility.c > CMakeFiles\__idf_efuse.dir\src\esp_efuse_utility.c.i

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_utility.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_efuse.dir/src/esp_efuse_utility.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\efuse\src\esp_efuse_utility.c -o CMakeFiles\__idf_efuse.dir\src\esp_efuse_utility.c.s

# Object files for target __idf_efuse
__idf_efuse_OBJECTS = \
"CMakeFiles/__idf_efuse.dir/esp32/esp_efuse_table.c.obj" \
"CMakeFiles/__idf_efuse.dir/src/esp_efuse_api.c.obj" \
"CMakeFiles/__idf_efuse.dir/src/esp_efuse_fields.c.obj" \
"CMakeFiles/__idf_efuse.dir/src/esp_efuse_utility.c.obj"

# External object files for target __idf_efuse
__idf_efuse_EXTERNAL_OBJECTS =

esp-idf/efuse/libefuse.a: esp-idf/efuse/CMakeFiles/__idf_efuse.dir/esp32/esp_efuse_table.c.obj
esp-idf/efuse/libefuse.a: esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_api.c.obj
esp-idf/efuse/libefuse.a: esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_fields.c.obj
esp-idf/efuse/libefuse.a: esp-idf/efuse/CMakeFiles/__idf_efuse.dir/src/esp_efuse_utility.c.obj
esp-idf/efuse/libefuse.a: esp-idf/efuse/CMakeFiles/__idf_efuse.dir/build.make
esp-idf/efuse/libefuse.a: esp-idf/efuse/CMakeFiles/__idf_efuse.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libefuse.a"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && $(CMAKE_COMMAND) -P CMakeFiles\__idf_efuse.dir\cmake_clean_target.cmake
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\__idf_efuse.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/efuse/CMakeFiles/__idf_efuse.dir/build: esp-idf/efuse/libefuse.a

.PHONY : esp-idf/efuse/CMakeFiles/__idf_efuse.dir/build

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/clean:
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse && $(CMAKE_COMMAND) -P CMakeFiles\__idf_efuse.dir\cmake_clean.cmake
.PHONY : esp-idf/efuse/CMakeFiles/__idf_efuse.dir/clean

esp-idf/efuse/CMakeFiles/__idf_efuse.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\pawel\Desktop\Grant\Project\firmware C:\Users\pawel\Desktop\esp-idf\components\efuse C:\Users\pawel\Desktop\Grant\Project\firmware\CMake C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\efuse\CMakeFiles\__idf_efuse.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/efuse/CMakeFiles/__idf_efuse.dir/depend

