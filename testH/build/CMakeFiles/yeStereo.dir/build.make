# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/ProjectDesign1/testH

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/ProjectDesign1/testH/build

# Include any dependencies generated for this target.
include CMakeFiles/yeStereo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/yeStereo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/yeStereo.dir/flags.make

CMakeFiles/yeStereo.dir/main2.cpp.o: CMakeFiles/yeStereo.dir/flags.make
CMakeFiles/yeStereo.dir/main2.cpp.o: ../main2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/ProjectDesign1/testH/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/yeStereo.dir/main2.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yeStereo.dir/main2.cpp.o -c /home/ubuntu/ProjectDesign1/testH/main2.cpp

CMakeFiles/yeStereo.dir/main2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yeStereo.dir/main2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/ProjectDesign1/testH/main2.cpp > CMakeFiles/yeStereo.dir/main2.cpp.i

CMakeFiles/yeStereo.dir/main2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yeStereo.dir/main2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/ProjectDesign1/testH/main2.cpp -o CMakeFiles/yeStereo.dir/main2.cpp.s

CMakeFiles/yeStereo.dir/calibration.cpp.o: CMakeFiles/yeStereo.dir/flags.make
CMakeFiles/yeStereo.dir/calibration.cpp.o: ../calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/ProjectDesign1/testH/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/yeStereo.dir/calibration.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yeStereo.dir/calibration.cpp.o -c /home/ubuntu/ProjectDesign1/testH/calibration.cpp

CMakeFiles/yeStereo.dir/calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yeStereo.dir/calibration.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/ProjectDesign1/testH/calibration.cpp > CMakeFiles/yeStereo.dir/calibration.cpp.i

CMakeFiles/yeStereo.dir/calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yeStereo.dir/calibration.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/ProjectDesign1/testH/calibration.cpp -o CMakeFiles/yeStereo.dir/calibration.cpp.s

# Object files for target yeStereo
yeStereo_OBJECTS = \
"CMakeFiles/yeStereo.dir/main2.cpp.o" \
"CMakeFiles/yeStereo.dir/calibration.cpp.o"

# External object files for target yeStereo
yeStereo_EXTERNAL_OBJECTS =

yeStereo: CMakeFiles/yeStereo.dir/main2.cpp.o
yeStereo: CMakeFiles/yeStereo.dir/calibration.cpp.o
yeStereo: CMakeFiles/yeStereo.dir/build.make
yeStereo: CMakeFiles/yeStereo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/ProjectDesign1/testH/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable yeStereo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/yeStereo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/yeStereo.dir/build: yeStereo

.PHONY : CMakeFiles/yeStereo.dir/build

CMakeFiles/yeStereo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/yeStereo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/yeStereo.dir/clean

CMakeFiles/yeStereo.dir/depend:
	cd /home/ubuntu/ProjectDesign1/testH/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/ProjectDesign1/testH /home/ubuntu/ProjectDesign1/testH /home/ubuntu/ProjectDesign1/testH/build /home/ubuntu/ProjectDesign1/testH/build /home/ubuntu/ProjectDesign1/testH/build/CMakeFiles/yeStereo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/yeStereo.dir/depend

