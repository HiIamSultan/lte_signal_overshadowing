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
include srsenb/src/stack/CMakeFiles/srsenb_stack.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include srsenb/src/stack/CMakeFiles/srsenb_stack.dir/compiler_depend.make

# Include the progress variables for this target.
include srsenb/src/stack/CMakeFiles/srsenb_stack.dir/progress.make

# Include the compile flags for this target's objects.
include srsenb/src/stack/CMakeFiles/srsenb_stack.dir/flags.make

srsenb/src/stack/CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.o: srsenb/src/stack/CMakeFiles/srsenb_stack.dir/flags.make
srsenb/src/stack/CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.o: ../srsenb/src/stack/enb_stack_lte.cc
srsenb/src/stack/CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.o: srsenb/src/stack/CMakeFiles/srsenb_stack.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object srsenb/src/stack/CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.o"
	cd /home/cseuser/Desktop/Overshadow/build/srsenb/src/stack && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT srsenb/src/stack/CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.o -MF CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.o.d -o CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.o -c /home/cseuser/Desktop/Overshadow/srsenb/src/stack/enb_stack_lte.cc

srsenb/src/stack/CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.i"
	cd /home/cseuser/Desktop/Overshadow/build/srsenb/src/stack && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/srsenb/src/stack/enb_stack_lte.cc > CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.i

srsenb/src/stack/CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.s"
	cd /home/cseuser/Desktop/Overshadow/build/srsenb/src/stack && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/srsenb/src/stack/enb_stack_lte.cc -o CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.s

# Object files for target srsenb_stack
srsenb_stack_OBJECTS = \
"CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.o"

# External object files for target srsenb_stack
srsenb_stack_EXTERNAL_OBJECTS =

srsenb/src/stack/libsrsenb_stack.a: srsenb/src/stack/CMakeFiles/srsenb_stack.dir/enb_stack_lte.cc.o
srsenb/src/stack/libsrsenb_stack.a: srsenb/src/stack/CMakeFiles/srsenb_stack.dir/build.make
srsenb/src/stack/libsrsenb_stack.a: srsenb/src/stack/CMakeFiles/srsenb_stack.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libsrsenb_stack.a"
	cd /home/cseuser/Desktop/Overshadow/build/srsenb/src/stack && $(CMAKE_COMMAND) -P CMakeFiles/srsenb_stack.dir/cmake_clean_target.cmake
	cd /home/cseuser/Desktop/Overshadow/build/srsenb/src/stack && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/srsenb_stack.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
srsenb/src/stack/CMakeFiles/srsenb_stack.dir/build: srsenb/src/stack/libsrsenb_stack.a
.PHONY : srsenb/src/stack/CMakeFiles/srsenb_stack.dir/build

srsenb/src/stack/CMakeFiles/srsenb_stack.dir/clean:
	cd /home/cseuser/Desktop/Overshadow/build/srsenb/src/stack && $(CMAKE_COMMAND) -P CMakeFiles/srsenb_stack.dir/cmake_clean.cmake
.PHONY : srsenb/src/stack/CMakeFiles/srsenb_stack.dir/clean

srsenb/src/stack/CMakeFiles/srsenb_stack.dir/depend:
	cd /home/cseuser/Desktop/Overshadow/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cseuser/Desktop/Overshadow /home/cseuser/Desktop/Overshadow/srsenb/src/stack /home/cseuser/Desktop/Overshadow/build /home/cseuser/Desktop/Overshadow/build/srsenb/src/stack /home/cseuser/Desktop/Overshadow/build/srsenb/src/stack/CMakeFiles/srsenb_stack.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : srsenb/src/stack/CMakeFiles/srsenb_stack.dir/depend
