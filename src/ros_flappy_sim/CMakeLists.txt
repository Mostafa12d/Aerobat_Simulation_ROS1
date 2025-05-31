cmake_minimum_required(VERSION 3.0.2)
project(ros_flappy_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

# add_message_files(
#   FILES
#   CameraFrame.msg
#   Axes.msg
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge image_transport message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY src/mujoco_ros_bridge/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/mujoco_ros_bridge
)