cmake_minimum_required(VERSION 2.8.3)
project(saphari_task_executive)

find_package(catkin REQUIRED roscpp roslisp message_generation geometry_msgs shape_msgs)

add_message_files(
  FILES
  ObjectTypeDescription.msg
  ObjectInstanceDescription.msg
  SlotDescription.msg
  SlotGoalDescription.msg
  SlotStateDescription.msg
  TaskState.msg)

generate_messages(DEPENDENCIES geometry_msgs shape_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime geometry_msgs shape_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_executable(humans_tf_publisher
  src/${PROJECT_NAME}/humans_tf_publisher.cpp)

target_link_libraries(humans_tf_publisher ${catkin_LIBRARIES})

add_lisp_executable(humans_filter
  saphari-humans-processing
  saphari-humans-processing:main-humans-filter)

add_lisp_executable(humans_visualization
  saphari-humans-processing
  saphari-humans-processing:main-humans-visualization)
