cmake_minimum_required(VERSION 3.3)
project(testing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wc++11-narrowing -Wdeprecated-declarations")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


find_package(PkgConfig)

file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/../drone/lineFollowing/*.cpp)
file(GLOB_RECURSE SRC_FILES ${SRC_FILES} ${PROJECT_SOURCE_DIR}/../drone/lineFollowing/*.h)
set(SRC_FILES ${SRC_FILES} ${PROJECT_SOURCE_DIR}/testing.cpp)

message("${SRC_FILES}")

add_executable(testing ${SRC_FILES})

set(libraries ${libraries} ${OpenCV_LIBS})
target_link_libraries(testing ${libraries})