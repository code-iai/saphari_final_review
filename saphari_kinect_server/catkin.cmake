cmake_minimum_required(VERSION 2.8.3)
project(saphari_kinect_server)

find_package(catkin REQUIRED 
  roscpp
  tf
  nodelet
  openni_camera
  saphari_msgs 
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp nodelet openni_camera saphari_msgs geometry_msgs sensor_msgs)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)
find_package(Nite)

# TODO: get boost?

if(OpenNI_FOUND AND Nite_FOUND)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenNI_INCLUDEDIR} 
    ${Nite_INCLUDE_DIR})

    add_library(kinect_driver src/driver.cpp src/user_tracker.cpp)
    target_link_libraries(kinect_driver 
      ${catkin_LIBRARIES} ${OpenNI_LIBRARIES} ${Nite_LIBRARIES})
    
    add_library (kinect_nodelet SHARED src/kinect_nodelet.cpp)
    target_link_libraries(kinect_nodelet
      ${catkin_LIBRARIES} ${OpenNI_LIBRARIES} ${Nite_LIBRARIES})

    add_executable (kinect_driver_node src/kinect_node.cpp)
    target_link_libraries(kinect_driver_node
      ${catkin_LIBRARIES} ${OpenNI_LIBRARIES} ${Nite_LIBRARIES} kinect_driver)

 
elseif(Nite_FOUND)
  message(ERROR " OpenNI not found. Stopping.")
  return()
elseif(OpenNI_FOUND)
  message(ERROR " NITE not found. Stopping.")
  return()
endif()

