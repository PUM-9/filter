cmake_minimum_required(VERSION 3.7.1)
project(filter)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL 1.3 REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options filesystem REQUIRED)

message(STATUS "Setting up BOOST")
message(STATUS " Includes - ${Boost_INCLUDE_DIRS}")
message(STATUS " Library  - ${Boost_LIBRARY_DIRS}")

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(-DBOOST_LOG_DYN_LINK)

set(SOURCE_FILES src/main.cpp src/Filter.cpp src/include/Filter.h)

add_executable(filter ${SOURCE_FILES})

target_link_libraries(filter ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS filter DESTINATION bin)
