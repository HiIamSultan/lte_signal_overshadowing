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
include lib/src/srslog/CMakeFiles/srslog.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include lib/src/srslog/CMakeFiles/srslog.dir/compiler_depend.make

# Include the progress variables for this target.
include lib/src/srslog/CMakeFiles/srslog.dir/progress.make

# Include the compile flags for this target's objects.
include lib/src/srslog/CMakeFiles/srslog.dir/flags.make

lib/src/srslog/CMakeFiles/srslog.dir/backend_worker.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/flags.make
lib/src/srslog/CMakeFiles/srslog.dir/backend_worker.cpp.o: ../lib/src/srslog/backend_worker.cpp
lib/src/srslog/CMakeFiles/srslog.dir/backend_worker.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/src/srslog/CMakeFiles/srslog.dir/backend_worker.cpp.o"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/src/srslog/CMakeFiles/srslog.dir/backend_worker.cpp.o -MF CMakeFiles/srslog.dir/backend_worker.cpp.o.d -o CMakeFiles/srslog.dir/backend_worker.cpp.o -c /home/cseuser/Desktop/Overshadow/lib/src/srslog/backend_worker.cpp

lib/src/srslog/CMakeFiles/srslog.dir/backend_worker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srslog.dir/backend_worker.cpp.i"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/lib/src/srslog/backend_worker.cpp > CMakeFiles/srslog.dir/backend_worker.cpp.i

lib/src/srslog/CMakeFiles/srslog.dir/backend_worker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srslog.dir/backend_worker.cpp.s"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/lib/src/srslog/backend_worker.cpp -o CMakeFiles/srslog.dir/backend_worker.cpp.s

lib/src/srslog/CMakeFiles/srslog.dir/srslog.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/flags.make
lib/src/srslog/CMakeFiles/srslog.dir/srslog.cpp.o: ../lib/src/srslog/srslog.cpp
lib/src/srslog/CMakeFiles/srslog.dir/srslog.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object lib/src/srslog/CMakeFiles/srslog.dir/srslog.cpp.o"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/src/srslog/CMakeFiles/srslog.dir/srslog.cpp.o -MF CMakeFiles/srslog.dir/srslog.cpp.o.d -o CMakeFiles/srslog.dir/srslog.cpp.o -c /home/cseuser/Desktop/Overshadow/lib/src/srslog/srslog.cpp

lib/src/srslog/CMakeFiles/srslog.dir/srslog.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srslog.dir/srslog.cpp.i"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/lib/src/srslog/srslog.cpp > CMakeFiles/srslog.dir/srslog.cpp.i

lib/src/srslog/CMakeFiles/srslog.dir/srslog.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srslog.dir/srslog.cpp.s"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/lib/src/srslog/srslog.cpp -o CMakeFiles/srslog.dir/srslog.cpp.s

lib/src/srslog/CMakeFiles/srslog.dir/srslog_c.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/flags.make
lib/src/srslog/CMakeFiles/srslog.dir/srslog_c.cpp.o: ../lib/src/srslog/srslog_c.cpp
lib/src/srslog/CMakeFiles/srslog.dir/srslog_c.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object lib/src/srslog/CMakeFiles/srslog.dir/srslog_c.cpp.o"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/src/srslog/CMakeFiles/srslog.dir/srslog_c.cpp.o -MF CMakeFiles/srslog.dir/srslog_c.cpp.o.d -o CMakeFiles/srslog.dir/srslog_c.cpp.o -c /home/cseuser/Desktop/Overshadow/lib/src/srslog/srslog_c.cpp

lib/src/srslog/CMakeFiles/srslog.dir/srslog_c.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srslog.dir/srslog_c.cpp.i"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/lib/src/srslog/srslog_c.cpp > CMakeFiles/srslog.dir/srslog_c.cpp.i

lib/src/srslog/CMakeFiles/srslog.dir/srslog_c.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srslog.dir/srslog_c.cpp.s"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/lib/src/srslog/srslog_c.cpp -o CMakeFiles/srslog.dir/srslog_c.cpp.s

lib/src/srslog/CMakeFiles/srslog.dir/event_trace.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/flags.make
lib/src/srslog/CMakeFiles/srslog.dir/event_trace.cpp.o: ../lib/src/srslog/event_trace.cpp
lib/src/srslog/CMakeFiles/srslog.dir/event_trace.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object lib/src/srslog/CMakeFiles/srslog.dir/event_trace.cpp.o"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/src/srslog/CMakeFiles/srslog.dir/event_trace.cpp.o -MF CMakeFiles/srslog.dir/event_trace.cpp.o.d -o CMakeFiles/srslog.dir/event_trace.cpp.o -c /home/cseuser/Desktop/Overshadow/lib/src/srslog/event_trace.cpp

lib/src/srslog/CMakeFiles/srslog.dir/event_trace.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srslog.dir/event_trace.cpp.i"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/lib/src/srslog/event_trace.cpp > CMakeFiles/srslog.dir/event_trace.cpp.i

lib/src/srslog/CMakeFiles/srslog.dir/event_trace.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srslog.dir/event_trace.cpp.s"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/lib/src/srslog/event_trace.cpp -o CMakeFiles/srslog.dir/event_trace.cpp.s

lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/format.cc.o: lib/src/srslog/CMakeFiles/srslog.dir/flags.make
lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/format.cc.o: ../lib/src/srslog/bundled/fmt/format.cc
lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/format.cc.o: lib/src/srslog/CMakeFiles/srslog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/format.cc.o"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/format.cc.o -MF CMakeFiles/srslog.dir/bundled/fmt/format.cc.o.d -o CMakeFiles/srslog.dir/bundled/fmt/format.cc.o -c /home/cseuser/Desktop/Overshadow/lib/src/srslog/bundled/fmt/format.cc

lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/format.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srslog.dir/bundled/fmt/format.cc.i"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/lib/src/srslog/bundled/fmt/format.cc > CMakeFiles/srslog.dir/bundled/fmt/format.cc.i

lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/format.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srslog.dir/bundled/fmt/format.cc.s"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/lib/src/srslog/bundled/fmt/format.cc -o CMakeFiles/srslog.dir/bundled/fmt/format.cc.s

lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/os.cc.o: lib/src/srslog/CMakeFiles/srslog.dir/flags.make
lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/os.cc.o: ../lib/src/srslog/bundled/fmt/os.cc
lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/os.cc.o: lib/src/srslog/CMakeFiles/srslog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/os.cc.o"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/os.cc.o -MF CMakeFiles/srslog.dir/bundled/fmt/os.cc.o.d -o CMakeFiles/srslog.dir/bundled/fmt/os.cc.o -c /home/cseuser/Desktop/Overshadow/lib/src/srslog/bundled/fmt/os.cc

lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/os.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srslog.dir/bundled/fmt/os.cc.i"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/lib/src/srslog/bundled/fmt/os.cc > CMakeFiles/srslog.dir/bundled/fmt/os.cc.i

lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/os.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srslog.dir/bundled/fmt/os.cc.s"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/lib/src/srslog/bundled/fmt/os.cc -o CMakeFiles/srslog.dir/bundled/fmt/os.cc.s

lib/src/srslog/CMakeFiles/srslog.dir/formatters/json_formatter.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/flags.make
lib/src/srslog/CMakeFiles/srslog.dir/formatters/json_formatter.cpp.o: ../lib/src/srslog/formatters/json_formatter.cpp
lib/src/srslog/CMakeFiles/srslog.dir/formatters/json_formatter.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object lib/src/srslog/CMakeFiles/srslog.dir/formatters/json_formatter.cpp.o"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/src/srslog/CMakeFiles/srslog.dir/formatters/json_formatter.cpp.o -MF CMakeFiles/srslog.dir/formatters/json_formatter.cpp.o.d -o CMakeFiles/srslog.dir/formatters/json_formatter.cpp.o -c /home/cseuser/Desktop/Overshadow/lib/src/srslog/formatters/json_formatter.cpp

lib/src/srslog/CMakeFiles/srslog.dir/formatters/json_formatter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srslog.dir/formatters/json_formatter.cpp.i"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/lib/src/srslog/formatters/json_formatter.cpp > CMakeFiles/srslog.dir/formatters/json_formatter.cpp.i

lib/src/srslog/CMakeFiles/srslog.dir/formatters/json_formatter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srslog.dir/formatters/json_formatter.cpp.s"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/lib/src/srslog/formatters/json_formatter.cpp -o CMakeFiles/srslog.dir/formatters/json_formatter.cpp.s

lib/src/srslog/CMakeFiles/srslog.dir/formatters/text_formatter.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/flags.make
lib/src/srslog/CMakeFiles/srslog.dir/formatters/text_formatter.cpp.o: ../lib/src/srslog/formatters/text_formatter.cpp
lib/src/srslog/CMakeFiles/srslog.dir/formatters/text_formatter.cpp.o: lib/src/srslog/CMakeFiles/srslog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object lib/src/srslog/CMakeFiles/srslog.dir/formatters/text_formatter.cpp.o"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/src/srslog/CMakeFiles/srslog.dir/formatters/text_formatter.cpp.o -MF CMakeFiles/srslog.dir/formatters/text_formatter.cpp.o.d -o CMakeFiles/srslog.dir/formatters/text_formatter.cpp.o -c /home/cseuser/Desktop/Overshadow/lib/src/srslog/formatters/text_formatter.cpp

lib/src/srslog/CMakeFiles/srslog.dir/formatters/text_formatter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srslog.dir/formatters/text_formatter.cpp.i"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cseuser/Desktop/Overshadow/lib/src/srslog/formatters/text_formatter.cpp > CMakeFiles/srslog.dir/formatters/text_formatter.cpp.i

lib/src/srslog/CMakeFiles/srslog.dir/formatters/text_formatter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srslog.dir/formatters/text_formatter.cpp.s"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cseuser/Desktop/Overshadow/lib/src/srslog/formatters/text_formatter.cpp -o CMakeFiles/srslog.dir/formatters/text_formatter.cpp.s

# Object files for target srslog
srslog_OBJECTS = \
"CMakeFiles/srslog.dir/backend_worker.cpp.o" \
"CMakeFiles/srslog.dir/srslog.cpp.o" \
"CMakeFiles/srslog.dir/srslog_c.cpp.o" \
"CMakeFiles/srslog.dir/event_trace.cpp.o" \
"CMakeFiles/srslog.dir/bundled/fmt/format.cc.o" \
"CMakeFiles/srslog.dir/bundled/fmt/os.cc.o" \
"CMakeFiles/srslog.dir/formatters/json_formatter.cpp.o" \
"CMakeFiles/srslog.dir/formatters/text_formatter.cpp.o"

# External object files for target srslog
srslog_EXTERNAL_OBJECTS =

lib/src/srslog/libsrslog.a: lib/src/srslog/CMakeFiles/srslog.dir/backend_worker.cpp.o
lib/src/srslog/libsrslog.a: lib/src/srslog/CMakeFiles/srslog.dir/srslog.cpp.o
lib/src/srslog/libsrslog.a: lib/src/srslog/CMakeFiles/srslog.dir/srslog_c.cpp.o
lib/src/srslog/libsrslog.a: lib/src/srslog/CMakeFiles/srslog.dir/event_trace.cpp.o
lib/src/srslog/libsrslog.a: lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/format.cc.o
lib/src/srslog/libsrslog.a: lib/src/srslog/CMakeFiles/srslog.dir/bundled/fmt/os.cc.o
lib/src/srslog/libsrslog.a: lib/src/srslog/CMakeFiles/srslog.dir/formatters/json_formatter.cpp.o
lib/src/srslog/libsrslog.a: lib/src/srslog/CMakeFiles/srslog.dir/formatters/text_formatter.cpp.o
lib/src/srslog/libsrslog.a: lib/src/srslog/CMakeFiles/srslog.dir/build.make
lib/src/srslog/libsrslog.a: lib/src/srslog/CMakeFiles/srslog.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cseuser/Desktop/Overshadow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX static library libsrslog.a"
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && $(CMAKE_COMMAND) -P CMakeFiles/srslog.dir/cmake_clean_target.cmake
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/srslog.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/src/srslog/CMakeFiles/srslog.dir/build: lib/src/srslog/libsrslog.a
.PHONY : lib/src/srslog/CMakeFiles/srslog.dir/build

lib/src/srslog/CMakeFiles/srslog.dir/clean:
	cd /home/cseuser/Desktop/Overshadow/build/lib/src/srslog && $(CMAKE_COMMAND) -P CMakeFiles/srslog.dir/cmake_clean.cmake
.PHONY : lib/src/srslog/CMakeFiles/srslog.dir/clean

lib/src/srslog/CMakeFiles/srslog.dir/depend:
	cd /home/cseuser/Desktop/Overshadow/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cseuser/Desktop/Overshadow /home/cseuser/Desktop/Overshadow/lib/src/srslog /home/cseuser/Desktop/Overshadow/build /home/cseuser/Desktop/Overshadow/build/lib/src/srslog /home/cseuser/Desktop/Overshadow/build/lib/src/srslog/CMakeFiles/srslog.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/src/srslog/CMakeFiles/srslog.dir/depend
