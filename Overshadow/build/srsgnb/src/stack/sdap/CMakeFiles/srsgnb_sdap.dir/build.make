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
CMAKE_SOURCE_DIR = /home/cseuser/Desktop/Overshadow

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cseuser/Desktop/Overshadow/build

# Include any dependencies generated for this target.
include srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/compiler_depend.make

# Include the progress variables for this target.
include srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/progress.make

# Include the compile flags for this target's objects.
include srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/flags.make

srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/sdap.cc.o: srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/flags.make
srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/sdap.cc.o: ../srsgnb/src/stack/sdap/sdap.cc
srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/sdap.cc.o: srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/sdap.cc.o"
	cd /home/cseuser/Desktop/Overshadow/build/srsgnb/src/stack/sdap && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/sdap.cc.o -MF CMakeFiles/srsgnb_sdap.dir/sdap.cc.o.d -o CMakeFiles/srsgnb_sdap.dir/sdap.cc.o -c /home/cseuser/Desktop/Overshadow/srsgnb/src/stack/sdap/sdap.cc

srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/sdap.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srsgnb_sdap.dir/sdap.cc.i"
	cd /home/cseuser/Desktop/Overshadow/build/srsgnb/src/stack/sdap && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/srsgnb/src/stack/sdap/sdap.cc > CMakeFiles/srsgnb_sdap.dir/sdap.cc.i

srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/sdap.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srsgnb_sdap.dir/sdap.cc.s"
	cd /home/cseuser/Desktop/Overshadow/build/srsgnb/src/stack/sdap && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/srsgnb/src/stack/sdap/sdap.cc -o CMakeFiles/srsgnb_sdap.dir/sdap.cc.s

# Object files for target srsgnb_sdap
srsgnb_sdap_OBJECTS = \
"CMakeFiles/srsgnb_sdap.dir/sdap.cc.o"

# External object files for target srsgnb_sdap
srsgnb_sdap_EXTERNAL_OBJECTS =

srsgnb/src/stack/sdap/libsrsgnb_sdap.a: srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/sdap.cc.o
srsgnb/src/stack/sdap/libsrsgnb_sdap.a: srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/build.make
srsgnb/src/stack/sdap/libsrsgnb_sdap.a: srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libsrsgnb_sdap.a"
	cd /home/cseuser/Desktop/Overshadow/build/srsgnb/src/stack/sdap && $(CMAKE_COMMAND) -P CMakeFiles/srsgnb_sdap.dir/cmake_clean_target.cmake
	cd /home/cseuser/Desktop/Overshadow/build/srsgnb/src/stack/sdap && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/srsgnb_sdap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/build: srsgnb/src/stack/sdap/libsrsgnb_sdap.a
.PHONY : srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/build

srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/clean:
	cd /home/cseuser/Desktop/Overshadow/build/srsgnb/src/stack/sdap && $(CMAKE_COMMAND) -P CMakeFiles/srsgnb_sdap.dir/cmake_clean.cmake
.PHONY : srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/clean

srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/depend:
	cd /home/cseuser/Desktop/Overshadow/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cseuser/Desktop/Overshadow /home/cseuser/Desktop/Overshadow/srsgnb/src/stack/sdap /home/cseuser/Desktop/Overshadow/build /home/cseuser/Desktop/Overshadow/build/srsgnb/src/stack/sdap /home/cseuser/Desktop/Overshadow/build/srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : srsgnb/src/stack/sdap/CMakeFiles/srsgnb_sdap.dir/depend

