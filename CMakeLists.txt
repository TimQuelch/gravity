cmake_minimum_required(VERSION 3.7)
project(gravity VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O2 -pipe")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-inconsistent-missing-override")

include_directories(include)

add_subdirectory(src)
add_subdirectory(doc)

