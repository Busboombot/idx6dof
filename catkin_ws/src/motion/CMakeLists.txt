cmake_minimum_required(VERSION 2.8.3)
project(motion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  stepper
)

catkin_package(
   LIBRARIES motion
   CATKIN_DEPENDS rospy std_msgs
   DEPENDS stepper
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

