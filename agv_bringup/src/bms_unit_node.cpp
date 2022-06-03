#include <ros/ros.h>
#include <Bms/bms.hpp>


int main(int argc, char** argv) {
  
  ros::init(argc, argv, "bms_unit_node");
  ros::NodeHandle nodeHandle;

  std::string port;
  int rate;

  ros::param::param<std::string>("/bms_unit_node/bms/port", port, "/dev/ttyS0");
  bms::Bms _bms(nodeHandle,port.c_str());

  ros::param::param<int>("/bms_unit_node/bms/publish_rate", rate, 1);
  ros::Rate loop_rate(rate);
  ROS_INFO_STREAM("port: " << port << "  publish_rate: " << rate);

  while (ros::ok())
  {
    _bms.update(); 
    loop_rate.sleep();
  }
  
  return 0;
}