cmake_minimum_required(VERSION 2.8.3)
project(humic_rl)

find_package(catkin REQUIRED COMPONENTS 
  rospy 
  std_msgs 
  trajectory_msgs 
  control_msgs 
  geometry_msgs 
  std_srvs 
  gazebo_msgs 
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs trajectory_msgs control_msgs geometry_msgs std_srvs  gazebo_msgs
)

install(DIRECTORY launch nodes src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/humicnavigation
  src/humic_navigation_env.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
