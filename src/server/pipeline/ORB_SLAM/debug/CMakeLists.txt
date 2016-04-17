cmake_minimum_required(VERSION 2.4.6)
project( ProbabilityMapping )
find_package( OpenCV REQUIRED )
add_executable( ${PROJECT_NAME}
    src/main.cc
    src/KeyFrame.cc
    src/ProbabilityMapping.cc
    src/LocalMapping.cc
)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

include_directories( ${PROJECT_SOURCE_DIR}/inc )

