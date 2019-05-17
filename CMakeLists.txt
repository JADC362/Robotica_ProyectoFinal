cmake_minimum_required(VERSION 2.8.3)
project(Robot5)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  message_generation
)

#Generate messages in the 'msg' folder
add_message_files(
  FILES
  RobotPos.msg
  ObstacleP.msg
  MotorVels.msg
  ObstaclesPos.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy std_msgs message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)