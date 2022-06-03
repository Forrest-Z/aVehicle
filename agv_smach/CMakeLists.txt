cmake_minimum_required(VERSION 3.0.2)
#cmake_minimum_required(VERSION 2.8.3)
project(agv_smach)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  laser_geometry
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  GuiStatus.msg
)

add_service_files(
  FILES 
  TaskService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)
include_directories(include ${catkin_INCLUDE_DIRS} $ENV{EIGEN3_INCLUDE_DIR})


add_executable(laser_to_pcl_transform src/agv_smach/laser_to_pcl_transform.cpp)

target_link_libraries(laser_to_pcl_transform
  ${catkin_LIBRARIES}
)


add_dependencies(laser_to_pcl_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


foreach(dir launch src srv params)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
