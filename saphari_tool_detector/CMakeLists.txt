cmake_minimum_required(VERSION 2.8.3)
project(saphari_tool_detector)

################################################
## Check for c++11 support                    ##
################################################
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

################################################
## Dependencies                               ##
################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib tf tf_conversions rostime sensor_msgs std_msgs geometry_msgs
  cv_bridge image_transport message_filters message_generation
)
find_package(cmake_modules QUIET)

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Tool.msg
)

add_service_files(
  FILES
  DetectTools.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES saphari_tools
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE found_files)
string(REPLACE "\n" ";" found_files_list ${found_files})
add_custom_target(${PROJECT_NAME}_additional_files SOURCES ${found_files_list})

add_definitions(-DDATA_PATH="${PROJECT_SOURCE_DIR}/data/")

## Shared library for saphari tool perception
add_library(saphari_tool_detection
  src/receiver.cpp
  src/perception.cpp
  src/visualizer.cpp
  src/intersection.cpp
)
target_link_libraries(saphari_tool_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}
)

## Saphari detection node
add_executable(detect_tools src/detect_tools.cpp)
target_link_libraries(detect_tools saphari_tool_detection)
add_dependencies(detect_tools ${catkin_EXPORTED_TARGETS})

## Saphari training node
add_executable(train_tool src/train_tool.cpp)
target_link_libraries(train_tool saphari_tool_detection)

## Saphari training node
add_executable(train_confidence src/train_confidence.cpp)
target_link_libraries(train_confidence saphari_tool_detection)
