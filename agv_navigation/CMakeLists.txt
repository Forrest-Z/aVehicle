cmake_minimum_required(VERSION 3.0.2)
project(agv_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/warehouse_layer.cpp
)

add_message_files(
  FILES
  localization_control/CarStateMsgs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES racecar_launch
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    razor_imu_9dof
    tf
    tf2_ros
    urg_node
    laser_scan_matcher
    rosbag
    rostopic
    rviz
    roscpp
    gmapping
    hector_mapping
    robot_pose_ekf
    message_runtime
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ../Drivers/udp_driver
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(initial_pose src/initial_pose.cpp)
target_link_libraries(initial_pose ${catkin_LIBRARIES})

add_executable(publish_odom src/publish_odom.cpp)
target_link_libraries(publish_odom ${catkin_LIBRARIES})

add_executable(imu_control src/localization_control/imu_control.cpp)
target_link_libraries(imu_control ${catkin_LIBRARIES})

add_executable(car_state src/localization_control/car_state.cpp)
target_link_libraries(car_state ${catkin_LIBRARIES})

add_executable(amcl_trigger src/localization_control/amcl_trigger.cpp)
target_link_libraries(amcl_trigger ${catkin_LIBRARIES})

add_executable(optitrack 
src/optitrack/optitrack.cpp

../Drivers/udp_driver/udp_client_server.cpp
)

target_link_libraries(optitrack ${catkin_LIBRARIES})