cmake_minimum_required(VERSION 3.0.2)
project(my_robot_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)