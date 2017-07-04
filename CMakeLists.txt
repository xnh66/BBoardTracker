cmake_minimum_required(VERSION 2.8)

SET(CMAKE_BUILD_TYPE "Debug")

project(SYAR)

SET(CMAKE_C_COMPILER g++)
add_definitions(-std=c++11)

set(OpenCV_DIR /usr/local/opencv3/share/OpenCV)
find_package(OpenCV 3 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}

)

add_library(${PROJECT_NAME} SHARED
AR_utils.cpp
ORBextractor.cpp
Ftrack.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)

add_executable(main main.cpp)
target_link_libraries(main ${PROJECT_NAME})


