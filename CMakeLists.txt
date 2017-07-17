cmake_minimum_required(VERSION 2.8)

SET(CMAKE_BUILD_TYPE "Debug")

project(BBOARDTRACKER)

SET(CMAKE_C_COMPILER g++)
add_definitions(-std=c++11)

#set(OpenCV_DIR /usr/local/opencv3/share/OpenCV)
#find_package(OpenCV 2.4.9 REQUIRED)
find_package(OpenCV 3.1.0 REQUIRED PATHS /usr/local/opencv310/share)
include_directories(/usr/local/opencv310/include)

include_directories(
${PROJECT_SOURCE_DIR/src}
)

add_library(${PROJECT_NAME} SHARED
src/AR_utils.cpp
src/ORBextractor.cpp
src/FTrack.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME})


