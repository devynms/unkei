# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros

# Include any dependencies generated for this target.
include CMakeFiles/mesher.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mesher.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mesher.dir/flags.make

CMakeFiles/mesher.dir/src/Mesher.cpp.o: CMakeFiles/mesher.dir/flags.make
CMakeFiles/mesher.dir/src/Mesher.cpp.o: src/Mesher.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mesher.dir/src/Mesher.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mesher.dir/src/Mesher.cpp.o -c /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros/src/Mesher.cpp

CMakeFiles/mesher.dir/src/Mesher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mesher.dir/src/Mesher.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros/src/Mesher.cpp > CMakeFiles/mesher.dir/src/Mesher.cpp.i

CMakeFiles/mesher.dir/src/Mesher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mesher.dir/src/Mesher.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros/src/Mesher.cpp -o CMakeFiles/mesher.dir/src/Mesher.cpp.s

CMakeFiles/mesher.dir/src/Mesher.cpp.o.requires:
.PHONY : CMakeFiles/mesher.dir/src/Mesher.cpp.o.requires

CMakeFiles/mesher.dir/src/Mesher.cpp.o.provides: CMakeFiles/mesher.dir/src/Mesher.cpp.o.requires
	$(MAKE) -f CMakeFiles/mesher.dir/build.make CMakeFiles/mesher.dir/src/Mesher.cpp.o.provides.build
.PHONY : CMakeFiles/mesher.dir/src/Mesher.cpp.o.provides

CMakeFiles/mesher.dir/src/Mesher.cpp.o.provides.build: CMakeFiles/mesher.dir/src/Mesher.cpp.o

CMakeFiles/mesher.dir/src/MesherExec.cpp.o: CMakeFiles/mesher.dir/flags.make
CMakeFiles/mesher.dir/src/MesherExec.cpp.o: src/MesherExec.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mesher.dir/src/MesherExec.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mesher.dir/src/MesherExec.cpp.o -c /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros/src/MesherExec.cpp

CMakeFiles/mesher.dir/src/MesherExec.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mesher.dir/src/MesherExec.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros/src/MesherExec.cpp > CMakeFiles/mesher.dir/src/MesherExec.cpp.i

CMakeFiles/mesher.dir/src/MesherExec.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mesher.dir/src/MesherExec.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros/src/MesherExec.cpp -o CMakeFiles/mesher.dir/src/MesherExec.cpp.s

CMakeFiles/mesher.dir/src/MesherExec.cpp.o.requires:
.PHONY : CMakeFiles/mesher.dir/src/MesherExec.cpp.o.requires

CMakeFiles/mesher.dir/src/MesherExec.cpp.o.provides: CMakeFiles/mesher.dir/src/MesherExec.cpp.o.requires
	$(MAKE) -f CMakeFiles/mesher.dir/build.make CMakeFiles/mesher.dir/src/MesherExec.cpp.o.provides.build
.PHONY : CMakeFiles/mesher.dir/src/MesherExec.cpp.o.provides

CMakeFiles/mesher.dir/src/MesherExec.cpp.o.provides.build: CMakeFiles/mesher.dir/src/MesherExec.cpp.o

# Object files for target mesher
mesher_OBJECTS = \
"CMakeFiles/mesher.dir/src/Mesher.cpp.o" \
"CMakeFiles/mesher.dir/src/MesherExec.cpp.o"

# External object files for target mesher
mesher_EXTERNAL_OBJECTS =

mesher: CMakeFiles/mesher.dir/src/Mesher.cpp.o
mesher: CMakeFiles/mesher.dir/src/MesherExec.cpp.o
mesher: CMakeFiles/mesher.dir/build.make
mesher: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libboost_system.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_thread.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
mesher: /usr/lib/x86_64-linux-gnu/libpthread.so
mesher: /usr/lib/libpcl_common.so
mesher: /usr/lib/libpcl_octree.so
mesher: /usr/lib/libOpenNI.so
mesher: /usr/lib/libvtkCommon.so.5.8.0
mesher: /usr/lib/libvtkRendering.so.5.8.0
mesher: /usr/lib/libvtkHybrid.so.5.8.0
mesher: /usr/lib/libvtkCharts.so.5.8.0
mesher: /usr/lib/libpcl_io.so
mesher: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
mesher: /usr/lib/libpcl_kdtree.so
mesher: /usr/lib/libpcl_search.so
mesher: /usr/lib/libpcl_sample_consensus.so
mesher: /usr/lib/libpcl_filters.so
mesher: /usr/lib/libpcl_features.so
mesher: /usr/lib/libpcl_keypoints.so
mesher: /usr/lib/libpcl_segmentation.so
mesher: /usr/lib/libpcl_visualization.so
mesher: /usr/lib/libpcl_outofcore.so
mesher: /usr/lib/libpcl_registration.so
mesher: /usr/lib/libpcl_recognition.so
mesher: /usr/lib/x86_64-linux-gnu/libqhull.so
mesher: /usr/lib/libpcl_surface.so
mesher: /usr/lib/libpcl_people.so
mesher: /usr/lib/libpcl_tracking.so
mesher: /usr/lib/libpcl_apps.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_system.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_thread.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
mesher: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
mesher: /usr/lib/x86_64-linux-gnu/libpthread.so
mesher: /usr/lib/x86_64-linux-gnu/libqhull.so
mesher: /usr/lib/libOpenNI.so
mesher: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
mesher: /usr/lib/libvtkCommon.so.5.8.0
mesher: /usr/lib/libvtkRendering.so.5.8.0
mesher: /usr/lib/libvtkHybrid.so.5.8.0
mesher: /usr/lib/libvtkCharts.so.5.8.0
mesher: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
mesher: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
mesher: /usr/lib/libpcl_common.so
mesher: /usr/lib/libpcl_octree.so
mesher: /usr/lib/libpcl_io.so
mesher: /usr/lib/libpcl_kdtree.so
mesher: /usr/lib/libpcl_search.so
mesher: /usr/lib/libpcl_sample_consensus.so
mesher: /usr/lib/libpcl_filters.so
mesher: /usr/lib/libpcl_features.so
mesher: /usr/lib/libpcl_keypoints.so
mesher: /usr/lib/libpcl_segmentation.so
mesher: /usr/lib/libpcl_visualization.so
mesher: /usr/lib/libpcl_outofcore.so
mesher: /usr/lib/libpcl_registration.so
mesher: /usr/lib/libpcl_recognition.so
mesher: /usr/lib/libpcl_surface.so
mesher: /usr/lib/libpcl_people.so
mesher: /usr/lib/libpcl_tracking.so
mesher: /usr/lib/libpcl_apps.so
mesher: /usr/lib/libvtkViews.so.5.8.0
mesher: /usr/lib/libvtkInfovis.so.5.8.0
mesher: /usr/lib/libvtkWidgets.so.5.8.0
mesher: /usr/lib/libvtkHybrid.so.5.8.0
mesher: /usr/lib/libvtkParallel.so.5.8.0
mesher: /usr/lib/libvtkVolumeRendering.so.5.8.0
mesher: /usr/lib/libvtkRendering.so.5.8.0
mesher: /usr/lib/libvtkGraphics.so.5.8.0
mesher: /usr/lib/libvtkImaging.so.5.8.0
mesher: /usr/lib/libvtkIO.so.5.8.0
mesher: /usr/lib/libvtkFiltering.so.5.8.0
mesher: /usr/lib/libvtkCommon.so.5.8.0
mesher: /usr/lib/libvtksys.so.5.8.0
mesher: CMakeFiles/mesher.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable mesher"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mesher.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mesher.dir/build: mesher
.PHONY : CMakeFiles/mesher.dir/build

CMakeFiles/mesher.dir/requires: CMakeFiles/mesher.dir/src/Mesher.cpp.o.requires
CMakeFiles/mesher.dir/requires: CMakeFiles/mesher.dir/src/MesherExec.cpp.o.requires
.PHONY : CMakeFiles/mesher.dir/requires

CMakeFiles/mesher.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mesher.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mesher.dir/clean

CMakeFiles/mesher.dir/depend:
	cd /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM_unros/CMakeFiles/mesher.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mesher.dir/depend
