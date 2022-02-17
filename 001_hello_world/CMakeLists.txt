cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(HELLO_WORLD)

# Using C++ 20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# PCL Settings
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(test src/main.cpp)
target_link_libraries(test ${PCL_LIBRARIES})