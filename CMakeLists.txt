cmake_minimum_required(VERSION 3.6)
project(filter)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.3 REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES src/main.cpp src/Filter.cpp src/include/Filter.h)
add_executable(filter ${SOURCE_FILES})

target_link_libraries(filter ${PCL_LIBRARIES} ${Boost_LIBRARIES})
