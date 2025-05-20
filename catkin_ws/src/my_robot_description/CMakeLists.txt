cmake_minimum_required(VERSION 3.0.2)
project(my_robot_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)