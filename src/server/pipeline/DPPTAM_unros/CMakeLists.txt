# attempt at building without ros

cmake_minimum_required(VERSION 2.8.3)
project(dpptam_noros)
add_definitions(-std=c++11 -Werror=return-type ${PCL_DEFINITIONS})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(cmake_modules REQUIRED)
find_package(PCL 1.2 REQUIRED)
#find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport cv_bridge tf pcl_ros)


set(CMAKE_BUILD_TYPE Debug)

#catkin_package(CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge tf)

set(BASEDIR ./)

set(DENSE_SOURCE_FILES     
		${BASEDIR}src/SemiDenseMapping.cpp
                ${BASEDIR}src/DenseMapping.cpp
                ${BASEDIR}src/SemiDenseTracking.cpp
                ${BASEDIR}src/vo_system.cpp
                ${BASEDIR}src/superpixel.cpp
		${BASEDIR}src/main.cpp
                ${BASEDIR}src/PlyListener.cpp
#               ${BASEDIR}src/Mesher.cpp
)


set(DENSE_HEADER_FILES  
		${BASEDIR}include/dpptam/SemiDenseMapping.h
                ${BASEDIR}include/dpptam/DenseMapping.h
                ${BASEDIR}include/dpptam/SemiDenseTracking.h
                ${BASEDIR}include/dpptam/vo_system.h
                ${BASEDIR}include/dpptam/superpixel.h
                ${BASEDIR}include/dpptam/PlyListener.h
#               ${BASEDIR}include/dpptam/Mesher.h
                ${BASEDIR}include/dpptam/MainWrapper.h
)

set(MESHER_SOURCE_FILES
                ${BASEDIR}src/Mesher.cpp
                ${BASEDIR}src/MesherExec.cpp
)
set(MESHER_HEADER_FILES
                ${BASEDIR}include/dpptam/Mesher.h
)

find_package(OpenCV REQUIRED)
include_directories(
  ${BASEDIR}include 
  #${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
#  LIBRARIES imu_3dm_gx3
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
find_package(OpenMP)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



option(OPTIMIZATION "Enable optimization" TRUE)
if(OPTIMIZATION)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
message("OPTIMIZATION FLAG ACTIVATED")
else()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
message("NO OPTIMIZATION FLAGS")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x ")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra -pedantic  -Wcast-qual -Wpointer-arith -Winit-self -Wswitch-default -Wmissing-include-dirs -Wold-style-cast -Wnon-virtual-dtor -Wshadow -Wno-unknown-pragmas")

link_directories(${PCL_LIBRARY_DIRS})
###########
## Build ##
###########
add_executable(dpptam_noros ${DENSE_HEADER_FILES} ${DENSE_SOURCE_FILES})
target_link_libraries(dpptam_noros ${OpenCV_LIBS} ${PCL_LIBRARIES})#${catkin_LIBRARIES})

add_executable(mesher ${MESHER_HEADER_FILES} ${MESHER_SOURCE_FILES})
target_link_libraries(mesher ${OpenCV_LIBS} ${PCL_LIBRARIES})#${catkin_LIBRARIES})

#install(PROGRAMS
#  scripts/run_bag
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
