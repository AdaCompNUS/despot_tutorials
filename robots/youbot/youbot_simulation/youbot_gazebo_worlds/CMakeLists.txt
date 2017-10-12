cmake_minimum_required(VERSION 2.8.3)
project(youbot_gazebo_worlds)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY launch urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

