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
CMAKE_SOURCE_DIR = /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM

# Include any dependencies generated for this target.
include CMakeFiles/dpptam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dpptam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dpptam.dir/flags.make

CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o: CMakeFiles/dpptam.dir/flags.make
CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o: src/SemiDenseMapping.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o -c /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/SemiDenseMapping.cpp

CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/SemiDenseMapping.cpp > CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.i

CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/SemiDenseMapping.cpp -o CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.s

CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o.requires:
.PHONY : CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o.requires

CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o.provides: CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o.requires
	$(MAKE) -f CMakeFiles/dpptam.dir/build.make CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o.provides.build
.PHONY : CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o.provides

CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o.provides.build: CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o

CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o: CMakeFiles/dpptam.dir/flags.make
CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o: src/DenseMapping.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o -c /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/DenseMapping.cpp

CMakeFiles/dpptam.dir/src/DenseMapping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dpptam.dir/src/DenseMapping.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/DenseMapping.cpp > CMakeFiles/dpptam.dir/src/DenseMapping.cpp.i

CMakeFiles/dpptam.dir/src/DenseMapping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dpptam.dir/src/DenseMapping.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/DenseMapping.cpp -o CMakeFiles/dpptam.dir/src/DenseMapping.cpp.s

CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o.requires:
.PHONY : CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o.requires

CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o.provides: CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o.requires
	$(MAKE) -f CMakeFiles/dpptam.dir/build.make CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o.provides.build
.PHONY : CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o.provides

CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o.provides.build: CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o

CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o: CMakeFiles/dpptam.dir/flags.make
CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o: src/SemiDenseTracking.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o -c /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/SemiDenseTracking.cpp

CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/SemiDenseTracking.cpp > CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.i

CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/SemiDenseTracking.cpp -o CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.s

CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o.requires:
.PHONY : CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o.requires

CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o.provides: CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o.requires
	$(MAKE) -f CMakeFiles/dpptam.dir/build.make CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o.provides.build
.PHONY : CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o.provides

CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o.provides.build: CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o

CMakeFiles/dpptam.dir/src/vo_system.cpp.o: CMakeFiles/dpptam.dir/flags.make
CMakeFiles/dpptam.dir/src/vo_system.cpp.o: src/vo_system.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dpptam.dir/src/vo_system.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dpptam.dir/src/vo_system.cpp.o -c /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/vo_system.cpp

CMakeFiles/dpptam.dir/src/vo_system.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dpptam.dir/src/vo_system.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/vo_system.cpp > CMakeFiles/dpptam.dir/src/vo_system.cpp.i

CMakeFiles/dpptam.dir/src/vo_system.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dpptam.dir/src/vo_system.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/vo_system.cpp -o CMakeFiles/dpptam.dir/src/vo_system.cpp.s

CMakeFiles/dpptam.dir/src/vo_system.cpp.o.requires:
.PHONY : CMakeFiles/dpptam.dir/src/vo_system.cpp.o.requires

CMakeFiles/dpptam.dir/src/vo_system.cpp.o.provides: CMakeFiles/dpptam.dir/src/vo_system.cpp.o.requires
	$(MAKE) -f CMakeFiles/dpptam.dir/build.make CMakeFiles/dpptam.dir/src/vo_system.cpp.o.provides.build
.PHONY : CMakeFiles/dpptam.dir/src/vo_system.cpp.o.provides

CMakeFiles/dpptam.dir/src/vo_system.cpp.o.provides.build: CMakeFiles/dpptam.dir/src/vo_system.cpp.o

CMakeFiles/dpptam.dir/src/superpixel.cpp.o: CMakeFiles/dpptam.dir/flags.make
CMakeFiles/dpptam.dir/src/superpixel.cpp.o: src/superpixel.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dpptam.dir/src/superpixel.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dpptam.dir/src/superpixel.cpp.o -c /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/superpixel.cpp

CMakeFiles/dpptam.dir/src/superpixel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dpptam.dir/src/superpixel.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/superpixel.cpp > CMakeFiles/dpptam.dir/src/superpixel.cpp.i

CMakeFiles/dpptam.dir/src/superpixel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dpptam.dir/src/superpixel.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/superpixel.cpp -o CMakeFiles/dpptam.dir/src/superpixel.cpp.s

CMakeFiles/dpptam.dir/src/superpixel.cpp.o.requires:
.PHONY : CMakeFiles/dpptam.dir/src/superpixel.cpp.o.requires

CMakeFiles/dpptam.dir/src/superpixel.cpp.o.provides: CMakeFiles/dpptam.dir/src/superpixel.cpp.o.requires
	$(MAKE) -f CMakeFiles/dpptam.dir/build.make CMakeFiles/dpptam.dir/src/superpixel.cpp.o.provides.build
.PHONY : CMakeFiles/dpptam.dir/src/superpixel.cpp.o.provides

CMakeFiles/dpptam.dir/src/superpixel.cpp.o.provides.build: CMakeFiles/dpptam.dir/src/superpixel.cpp.o

CMakeFiles/dpptam.dir/src/main.cpp.o: CMakeFiles/dpptam.dir/flags.make
CMakeFiles/dpptam.dir/src/main.cpp.o: src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dpptam.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dpptam.dir/src/main.cpp.o -c /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/main.cpp

CMakeFiles/dpptam.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dpptam.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/main.cpp > CMakeFiles/dpptam.dir/src/main.cpp.i

CMakeFiles/dpptam.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dpptam.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/src/main.cpp -o CMakeFiles/dpptam.dir/src/main.cpp.s

CMakeFiles/dpptam.dir/src/main.cpp.o.requires:
.PHONY : CMakeFiles/dpptam.dir/src/main.cpp.o.requires

CMakeFiles/dpptam.dir/src/main.cpp.o.provides: CMakeFiles/dpptam.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/dpptam.dir/build.make CMakeFiles/dpptam.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/dpptam.dir/src/main.cpp.o.provides

CMakeFiles/dpptam.dir/src/main.cpp.o.provides.build: CMakeFiles/dpptam.dir/src/main.cpp.o

# Object files for target dpptam
dpptam_OBJECTS = \
"CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o" \
"CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o" \
"CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o" \
"CMakeFiles/dpptam.dir/src/vo_system.cpp.o" \
"CMakeFiles/dpptam.dir/src/superpixel.cpp.o" \
"CMakeFiles/dpptam.dir/src/main.cpp.o"

# External object files for target dpptam
dpptam_EXTERNAL_OBJECTS =

devel/lib/dpptam/dpptam: CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o
devel/lib/dpptam/dpptam: CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o
devel/lib/dpptam/dpptam: CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o
devel/lib/dpptam/dpptam: CMakeFiles/dpptam.dir/src/vo_system.cpp.o
devel/lib/dpptam/dpptam: CMakeFiles/dpptam.dir/src/superpixel.cpp.o
devel/lib/dpptam/dpptam: CMakeFiles/dpptam.dir/src/main.cpp.o
devel/lib/dpptam/dpptam: CMakeFiles/dpptam.dir/build.make
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libimage_transport.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libcv_bridge.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libpcl_ros_filters.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libpcl_ros_io.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libpcl_ros_tf.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_common.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_kdtree.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_octree.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_search.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_surface.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_sample_consensus.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_filters.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_features.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_segmentation.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_io.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_registration.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_keypoints.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_recognition.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_visualization.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_people.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_outofcore.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_tracking.so
devel/lib/dpptam/dpptam: /usr/lib/libpcl_apps.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libqhull.so
devel/lib/dpptam/dpptam: /usr/lib/libOpenNI.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
devel/lib/dpptam/dpptam: /usr/lib/libvtkCommon.so.5.8.0
devel/lib/dpptam/dpptam: /usr/lib/libvtkRendering.so.5.8.0
devel/lib/dpptam/dpptam: /usr/lib/libvtkHybrid.so.5.8.0
devel/lib/dpptam/dpptam: /usr/lib/libvtkCharts.so.5.8.0
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libnodeletlib.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libbondcpp.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libuuid.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libtinyxml.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libclass_loader.so
devel/lib/dpptam/dpptam: /usr/lib/libPocoFoundation.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libroslib.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/librosbag.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/librosbag_storage.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libroslz4.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/liblz4.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libtopic_tools.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libtf.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libtf2_ros.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libactionlib.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libmessage_filters.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libroscpp.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libxmlrpcpp.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libtf2.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libroscpp_serialization.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/librosconsole.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/librosconsole_log4cxx.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/librosconsole_backend_interface.so
devel/lib/dpptam/dpptam: /usr/lib/liblog4cxx.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/librostime.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/dpptam/dpptam: /opt/ros/indigo/lib/libcpp_common.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
devel/lib/dpptam/dpptam: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
devel/lib/dpptam/dpptam: CMakeFiles/dpptam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable devel/lib/dpptam/dpptam"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dpptam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dpptam.dir/build: devel/lib/dpptam/dpptam
.PHONY : CMakeFiles/dpptam.dir/build

CMakeFiles/dpptam.dir/requires: CMakeFiles/dpptam.dir/src/SemiDenseMapping.cpp.o.requires
CMakeFiles/dpptam.dir/requires: CMakeFiles/dpptam.dir/src/DenseMapping.cpp.o.requires
CMakeFiles/dpptam.dir/requires: CMakeFiles/dpptam.dir/src/SemiDenseTracking.cpp.o.requires
CMakeFiles/dpptam.dir/requires: CMakeFiles/dpptam.dir/src/vo_system.cpp.o.requires
CMakeFiles/dpptam.dir/requires: CMakeFiles/dpptam.dir/src/superpixel.cpp.o.requires
CMakeFiles/dpptam.dir/requires: CMakeFiles/dpptam.dir/src/main.cpp.o.requires
.PHONY : CMakeFiles/dpptam.dir/requires

CMakeFiles/dpptam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dpptam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dpptam.dir/clean

CMakeFiles/dpptam.dir/depend:
	cd /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM /home/josh/Workspace/unkei/src/server/pipeline/DPPTAM/CMakeFiles/dpptam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dpptam.dir/depend

