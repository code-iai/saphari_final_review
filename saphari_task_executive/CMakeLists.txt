cmake_minimum_required(VERSION 2.8.3)
project(saphari_task_executive)

find_package(catkin REQUIRED message_generation)

add_message_files(
  FILES
  State.msg)

add_service_files(
  FILES
  RequestState.srv)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime)
