cmake_minimum_required(VERSION 3.0.2)
project(my_turtlebot_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(CATKIN_DEPENDS rospy std_msgs)

catkin_install_python(PROGRAMS mover.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(my_turtlebot_sim.test)
endif()