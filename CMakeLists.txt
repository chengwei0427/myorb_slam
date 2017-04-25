cmake_minimum_required(VERSION 2.6)
project(feature_detect_adjust)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV 3 REQUIRED)
include_directories(${PROJECT_SOURCE_DIR} 
                    ${PROJECT_SOURCE_DIR}/include 
                    ${OpenCV_INCLUDE_DIRS})

add_executable(feature_detect_adjust src/pinhole_camera.cpp src/feature.cpp src/frame.cpp src/orb_extractor.cpp src/feature_detect_adjust.cpp)
target_link_libraries(feature_detect_adjust ${OpenCV_LIBS})
