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
include esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/depend.make

# Include the progress variables for this target.
include esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/flags.make

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/mqtt_client.c.obj: esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/flags.make
esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/mqtt_client.c.obj: C:/Users/pawel/Desktop/esp-idf/components/mqtt/esp-mqtt/mqtt_client.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/mqtt_client.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_mqtt.dir\esp-mqtt\mqtt_client.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\mqtt_client.c

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/mqtt_client.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_mqtt.dir/esp-mqtt/mqtt_client.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\mqtt_client.c > CMakeFiles\__idf_mqtt.dir\esp-mqtt\mqtt_client.c.i

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/mqtt_client.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_mqtt.dir/esp-mqtt/mqtt_client.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\mqtt_client.c -o CMakeFiles\__idf_mqtt.dir\esp-mqtt\mqtt_client.c.s

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_msg.c.obj: esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/flags.make
esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_msg.c.obj: C:/Users/pawel/Desktop/esp-idf/components/mqtt/esp-mqtt/lib/mqtt_msg.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_msg.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_mqtt.dir\esp-mqtt\lib\mqtt_msg.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\lib\mqtt_msg.c

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_msg.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_msg.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\lib\mqtt_msg.c > CMakeFiles\__idf_mqtt.dir\esp-mqtt\lib\mqtt_msg.c.i

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_msg.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_msg.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\lib\mqtt_msg.c -o CMakeFiles\__idf_mqtt.dir\esp-mqtt\lib\mqtt_msg.c.s

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_outbox.c.obj: esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/flags.make
esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_outbox.c.obj: C:/Users/pawel/Desktop/esp-idf/components/mqtt/esp-mqtt/lib/mqtt_outbox.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_outbox.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_mqtt.dir\esp-mqtt\lib\mqtt_outbox.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\lib\mqtt_outbox.c

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_outbox.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_outbox.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\lib\mqtt_outbox.c > CMakeFiles\__idf_mqtt.dir\esp-mqtt\lib\mqtt_outbox.c.i

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_outbox.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_outbox.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\lib\mqtt_outbox.c -o CMakeFiles\__idf_mqtt.dir\esp-mqtt\lib\mqtt_outbox.c.s

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/platform_esp32_idf.c.obj: esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/flags.make
esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/platform_esp32_idf.c.obj: C:/Users/pawel/Desktop/esp-idf/components/mqtt/esp-mqtt/lib/platform_esp32_idf.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/platform_esp32_idf.c.obj"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles\__idf_mqtt.dir\esp-mqtt\lib\platform_esp32_idf.c.obj   -c C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\lib\platform_esp32_idf.c

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/platform_esp32_idf.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/platform_esp32_idf.c.i"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\lib\platform_esp32_idf.c > CMakeFiles\__idf_mqtt.dir\esp-mqtt\lib\platform_esp32_idf.c.i

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/platform_esp32_idf.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/platform_esp32_idf.c.s"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && C:\Users\pawel\.espressif\tools\xtensa-esp32-elf\esp-2020r3-8.4.0\xtensa-esp32-elf\bin\xtensa-esp32-elf-gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\pawel\Desktop\esp-idf\components\mqtt\esp-mqtt\lib\platform_esp32_idf.c -o CMakeFiles\__idf_mqtt.dir\esp-mqtt\lib\platform_esp32_idf.c.s

# Object files for target __idf_mqtt
__idf_mqtt_OBJECTS = \
"CMakeFiles/__idf_mqtt.dir/esp-mqtt/mqtt_client.c.obj" \
"CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_msg.c.obj" \
"CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_outbox.c.obj" \
"CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/platform_esp32_idf.c.obj"

# External object files for target __idf_mqtt
__idf_mqtt_EXTERNAL_OBJECTS =

esp-idf/mqtt/libmqtt.a: esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/mqtt_client.c.obj
esp-idf/mqtt/libmqtt.a: esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_msg.c.obj
esp-idf/mqtt/libmqtt.a: esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/mqtt_outbox.c.obj
esp-idf/mqtt/libmqtt.a: esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/esp-mqtt/lib/platform_esp32_idf.c.obj
esp-idf/mqtt/libmqtt.a: esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/build.make
esp-idf/mqtt/libmqtt.a: esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libmqtt.a"
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && $(CMAKE_COMMAND) -P CMakeFiles\__idf_mqtt.dir\cmake_clean_target.cmake
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\__idf_mqtt.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/build: esp-idf/mqtt/libmqtt.a

.PHONY : esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/build

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/clean:
	cd /d C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt && $(CMAKE_COMMAND) -P CMakeFiles\__idf_mqtt.dir\cmake_clean.cmake
.PHONY : esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/clean

esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\pawel\Desktop\Grant\Project\firmware C:\Users\pawel\Desktop\esp-idf\components\mqtt C:\Users\pawel\Desktop\Grant\Project\firmware\CMake C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt C:\Users\pawel\Desktop\Grant\Project\firmware\CMake\esp-idf\mqtt\CMakeFiles\__idf_mqtt.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/mqtt/CMakeFiles/__idf_mqtt.dir/depend

