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
include lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/compiler_depend.make

# Include the progress variables for this target.
include lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/progress.make

# Include the compile flags for this target's objects.
include lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/flags.make

lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.o: lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/flags.make
lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.o: ../lib/examples/pdsch_enodeb_smd.c
lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.o: lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.o"
	cd /home/cseuser/Desktop/Overshadow/build/lib/examples && /usr/bin/ccache /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.o -MF CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.o.d -o CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.o -c /home/cseuser/Desktop/Overshadow/lib/examples/pdsch_enodeb_smd.c

lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.i"
	cd /home/cseuser/Desktop/Overshadow/build/lib/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/cseuser/Desktop/Overshadow/lib/examples/pdsch_enodeb_smd.c > CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.i

lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.s"
	cd /home/cseuser/Desktop/Overshadow/build/lib/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/cseuser/Desktop/Overshadow/lib/examples/pdsch_enodeb_smd.c -o CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.s

# Object files for target pdsch_enodeb_smd
pdsch_enodeb_smd_OBJECTS = \
"CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.o"

# External object files for target pdsch_enodeb_smd
pdsch_enodeb_smd_EXTERNAL_OBJECTS =

lib/examples/pdsch_enodeb_smd: lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/pdsch_enodeb_smd.c.o
lib/examples/pdsch_enodeb_smd: lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/build.make
lib/examples/pdsch_enodeb_smd: lib/src/phy/libsrsran_phy.a
lib/examples/pdsch_enodeb_smd: lib/src/common/libsrsran_common.a
lib/examples/pdsch_enodeb_smd: lib/src/phy/rf/libsrsran_rf.so.22.10.0
lib/examples/pdsch_enodeb_smd: lib/src/support/libsupport.a
lib/examples/pdsch_enodeb_smd: lib/src/srslog/libsrslog.a
lib/examples/pdsch_enodeb_smd: /usr/lib/x86_64-linux-gnu/libmbedcrypto.so
lib/examples/pdsch_enodeb_smd: lib/src/phy/rf/libsrsran_rf_utils.a
lib/examples/pdsch_enodeb_smd: lib/src/phy/libsrsran_phy.a
lib/examples/pdsch_enodeb_smd: /usr/lib/x86_64-linux-gnu/libfftw3f.so
lib/examples/pdsch_enodeb_smd: lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pdsch_enodeb_smd"
	cd /home/cseuser/Desktop/Overshadow/build/lib/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pdsch_enodeb_smd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/build: lib/examples/pdsch_enodeb_smd
.PHONY : lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/build

lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/clean:
	cd /home/cseuser/Desktop/Overshadow/build/lib/examples && $(CMAKE_COMMAND) -P CMakeFiles/pdsch_enodeb_smd.dir/cmake_clean.cmake
.PHONY : lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/clean

lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/depend:
	cd /home/cseuser/Desktop/Overshadow/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cseuser/Desktop/Overshadow /home/cseuser/Desktop/Overshadow/lib/examples /home/cseuser/Desktop/Overshadow/build /home/cseuser/Desktop/Overshadow/build/lib/examples /home/cseuser/Desktop/Overshadow/build/lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/examples/CMakeFiles/pdsch_enodeb_smd.dir/depend
