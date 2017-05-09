cmake_minimum_required(VERSION 3.3)
project(overall_project)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wc++11-narrowing -Wdeprecated-declarations")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")


add_subdirectory(drone)
add_subdirectory(testing_cv)