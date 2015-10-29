cmake_minimum_required(VERSION 2.8.3)
project(saphari_interactive_markers)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED 
  roscpp 
  interactive_markers 
  visualization_msgs 
  geometry_msgs)

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS})

add_executable(saphari_interactive_markers 
  src/saphari_interactive_markers/main.cpp)

target_link_libraries(saphari_interactive_markers
  ${catkin_LIBRARIES})
