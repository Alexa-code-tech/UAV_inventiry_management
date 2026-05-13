cmake_minimum_required(VERSION 3.0.2)
project(lidar_visual_fusion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  robot_localization
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
)

catkin_install_python(PROGRAMS
  src/inventory_manager.py
  src/qr_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
