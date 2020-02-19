cmake_minimum_required(VERSION 2.8.3)
project(stepper)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
   FILES
   MoveCommand.msg
   PositionReport.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  #LIBRARIES stepper
  CATKIN_DEPENDS roscpp rospy serial std_msgs sensor_msgs message_runtime
)


include_directories(
  CRCpp/inc
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/stepper.cpp src/messages.cpp)
target_link_libraries(${PROJECT_NAME}_node serial ${catkin_LIBRARIES}  )


add_executable(${PROJECT_NAME}_example src/serial_example.cc)
target_link_libraries(${PROJECT_NAME}_example serial )


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})










