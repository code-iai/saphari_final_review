cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_find_ros_package(saphari_msgs)
rosbuild_find_ros_package(geometry_msgs)

find_package(OpenNI)

find_package(OpenGL)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

rosbuild_add_boost_directories()

include(FindPkgConfig)
pkg_check_modules(OPENNI libopenni)

include_directories(${OPENNI_INCLUDEDIR} /usr/include/nite)
		    # /home/rmcpc0242/catkin_ws/devel/include)

link_directories(/home/rmcpc0242/catkin_ws/src/dlr_beasty/dlr_ard/lib/)

rosbuild_add_library (kinect_driver src/driver.cpp src/user_tracker.cpp)
target_link_libraries (kinect_driver OpenNI)

rosbuild_add_executable (kinect_driver_node src/kinect_node.cpp)
target_link_libraries (kinect_driver_node kinect_driver)

#rosbuild_add_executable (calibration src/kinect_calibration.cpp)
#target_link_libraries (calibration OpenNI ${OpenCV_LIBRARIES} -lglut)

#rosbuild_add_executable (calib src/calibration.cpp)
#target_link_libraries (calib OpenNI ${OpenCV_LIBRARIES} -lglut)

#rosbuild_add_executable (calib src/calib.cpp)
#target_link_libraries (calib OpenNI ${OpenCV_LIBRARIES} -lglut -lGL)

rosbuild_add_library (kinect_nodelet src/kinect_nodelet.cpp)
target_link_libraries (kinect_nodelet kinect_driver)

