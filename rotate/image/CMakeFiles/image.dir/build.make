# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vision/Desktop/cv/2016spring/rotate/image

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vision/Desktop/cv/2016spring/rotate/image

# Include any dependencies generated for this target.
include CMakeFiles/image.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/image.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/image.dir/flags.make

CMakeFiles/image.dir/main.cpp.o: CMakeFiles/image.dir/flags.make
CMakeFiles/image.dir/main.cpp.o: main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/vision/Desktop/cv/2016spring/rotate/image/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/image.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/image.dir/main.cpp.o -c /home/vision/Desktop/cv/2016spring/rotate/image/main.cpp

CMakeFiles/image.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/vision/Desktop/cv/2016spring/rotate/image/main.cpp > CMakeFiles/image.dir/main.cpp.i

CMakeFiles/image.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/vision/Desktop/cv/2016spring/rotate/image/main.cpp -o CMakeFiles/image.dir/main.cpp.s

CMakeFiles/image.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/image.dir/main.cpp.o.requires

CMakeFiles/image.dir/main.cpp.o.provides: CMakeFiles/image.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/image.dir/build.make CMakeFiles/image.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/image.dir/main.cpp.o.provides

CMakeFiles/image.dir/main.cpp.o.provides.build: CMakeFiles/image.dir/main.cpp.o

# Object files for target image
image_OBJECTS = \
"CMakeFiles/image.dir/main.cpp.o"

# External object files for target image
image_EXTERNAL_OBJECTS =

image: CMakeFiles/image.dir/main.cpp.o
image: CMakeFiles/image.dir/build.make
image: /usr/local/lib/libopencv_videostab.so.3.1.0
image: /usr/local/lib/libopencv_superres.so.3.1.0
image: /usr/local/lib/libopencv_stitching.so.3.1.0
image: /usr/local/lib/libopencv_shape.so.3.1.0
image: /usr/local/lib/libopencv_photo.so.3.1.0
image: /usr/local/lib/libopencv_cudastereo.so.3.1.0
image: /usr/local/lib/libopencv_cudaoptflow.so.3.1.0
image: /usr/local/lib/libopencv_cudaobjdetect.so.3.1.0
image: /usr/local/lib/libopencv_cudalegacy.so.3.1.0
image: /usr/local/lib/libopencv_cudaimgproc.so.3.1.0
image: /usr/local/lib/libopencv_cudafeatures2d.so.3.1.0
image: /usr/local/lib/libopencv_cudacodec.so.3.1.0
image: /usr/local/lib/libopencv_cudabgsegm.so.3.1.0
image: /usr/local/lib/libopencv_calib3d.so.3.1.0
image: /usr/local/lib/libopencv_cudawarping.so.3.1.0
image: /usr/local/lib/libopencv_objdetect.so.3.1.0
image: /usr/local/lib/libopencv_cudafilters.so.3.1.0
image: /usr/local/lib/libopencv_cudaarithm.so.3.1.0
image: /usr/local/lib/libopencv_features2d.so.3.1.0
image: /usr/local/lib/libopencv_ml.so.3.1.0
image: /usr/local/lib/libopencv_highgui.so.3.1.0
image: /usr/local/lib/libopencv_videoio.so.3.1.0
image: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
image: /usr/local/lib/libopencv_flann.so.3.1.0
image: /usr/local/lib/libopencv_video.so.3.1.0
image: /usr/local/lib/libopencv_imgproc.so.3.1.0
image: /usr/local/lib/libopencv_core.so.3.1.0
image: /usr/local/lib/libopencv_cudev.so.3.1.0
image: CMakeFiles/image.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable image"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/image.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/image.dir/build: image
.PHONY : CMakeFiles/image.dir/build

CMakeFiles/image.dir/requires: CMakeFiles/image.dir/main.cpp.o.requires
.PHONY : CMakeFiles/image.dir/requires

CMakeFiles/image.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/image.dir/cmake_clean.cmake
.PHONY : CMakeFiles/image.dir/clean

CMakeFiles/image.dir/depend:
	cd /home/vision/Desktop/cv/2016spring/rotate/image && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vision/Desktop/cv/2016spring/rotate/image /home/vision/Desktop/cv/2016spring/rotate/image /home/vision/Desktop/cv/2016spring/rotate/image /home/vision/Desktop/cv/2016spring/rotate/image /home/vision/Desktop/cv/2016spring/rotate/image/CMakeFiles/image.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/image.dir/depend

