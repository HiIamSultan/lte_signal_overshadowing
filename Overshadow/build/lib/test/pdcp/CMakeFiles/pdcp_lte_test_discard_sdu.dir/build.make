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
include lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/compiler_depend.make

# Include the progress variables for this target.
include lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/progress.make

# Include the compile flags for this target's objects.
include lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/flags.make

lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.o: lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/flags.make
lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.o: ../lib/test/pdcp/pdcp_lte_test_discard_sdu.cc
lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.o: lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.o"
	cd /home/cseuser/Desktop/Overshadow/build/lib/test/pdcp && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.o -MF CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.o.d -o CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.o -c /home/cseuser/Desktop/Overshadow/lib/test/pdcp/pdcp_lte_test_discard_sdu.cc

lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.i"
	cd /home/cseuser/Desktop/Overshadow/build/lib/test/pdcp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/lib/test/pdcp/pdcp_lte_test_discard_sdu.cc > CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.i

lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.s"
	cd /home/cseuser/Desktop/Overshadow/build/lib/test/pdcp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/lib/test/pdcp/pdcp_lte_test_discard_sdu.cc -o CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.s

# Object files for target pdcp_lte_test_discard_sdu
pdcp_lte_test_discard_sdu_OBJECTS = \
"CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.o"

# External object files for target pdcp_lte_test_discard_sdu
pdcp_lte_test_discard_sdu_EXTERNAL_OBJECTS =

lib/test/pdcp/pdcp_lte_test_discard_sdu: lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/pdcp_lte_test_discard_sdu.cc.o
lib/test/pdcp/pdcp_lte_test_discard_sdu: lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/build.make
lib/test/pdcp/pdcp_lte_test_discard_sdu: lib/src/pdcp/libsrsran_pdcp.a
lib/test/pdcp/pdcp_lte_test_discard_sdu: lib/src/common/libsrsran_common.a
lib/test/pdcp/pdcp_lte_test_discard_sdu: lib/src/phy/libsrsran_phy.a
lib/test/pdcp/pdcp_lte_test_discard_sdu: /usr/lib/x86_64-linux-gnu/libfftw3f.so
lib/test/pdcp/pdcp_lte_test_discard_sdu: lib/src/support/libsupport.a
lib/test/pdcp/pdcp_lte_test_discard_sdu: lib/src/srslog/libsrslog.a
lib/test/pdcp/pdcp_lte_test_discard_sdu: /usr/lib/x86_64-linux-gnu/libmbedcrypto.so
lib/test/pdcp/pdcp_lte_test_discard_sdu: lib/src/asn1/libsrsran_asn1.a
lib/test/pdcp/pdcp_lte_test_discard_sdu: lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pdcp_lte_test_discard_sdu"
	cd /home/cseuser/Desktop/Overshadow/build/lib/test/pdcp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pdcp_lte_test_discard_sdu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/build: lib/test/pdcp/pdcp_lte_test_discard_sdu
.PHONY : lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/build

lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/clean:
	cd /home/cseuser/Desktop/Overshadow/build/lib/test/pdcp && $(CMAKE_COMMAND) -P CMakeFiles/pdcp_lte_test_discard_sdu.dir/cmake_clean.cmake
.PHONY : lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/clean

lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/depend:
	cd /home/cseuser/Desktop/Overshadow/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cseuser/Desktop/Overshadow /home/cseuser/Desktop/Overshadow/lib/test/pdcp /home/cseuser/Desktop/Overshadow/build /home/cseuser/Desktop/Overshadow/build/lib/test/pdcp /home/cseuser/Desktop/Overshadow/build/lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/test/pdcp/CMakeFiles/pdcp_lte_test_discard_sdu.dir/depend

