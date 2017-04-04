cmake_minimum_required(VERSION 3.6)
project(filter)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES src/main.cpp src/Filter.cpp src/include/Filter.h)
add_executable(filter ${SOURCE_FILES})

target_link_libraries(filter ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
