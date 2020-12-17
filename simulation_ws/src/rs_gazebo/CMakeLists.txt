cmake_minimum_required(VERSION 2.8.3)
project(rs_gazebo)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS gazebo_ros  gazebo_ros_control rs_description rs_control
)

install(FILES
  launch/HQ.launch
  launch/robot.launch
  worlds/ss_hq4.world
  worlds/stairs_only.world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
