cmake_minimum_required(VERSION 3.0.2)
project(agv_testing)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  move_base_msgs
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
   move_base_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch src)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()