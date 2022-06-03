/*
This code is used for publishing initial position from 
odom topic to /initialpose topic to use in /amcl node. 
*/

#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

float x, y, w, z;

/*Subscriber for  odom topic*/
void odomCallback(const nav_msgs::Odometry msg) {
  x = msg.pose.pose.position.x;
  y = msg.pose.pose.position.y;
  w = msg.pose.pose.orientation.w;
  z = msg.pose.pose.orientation.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "initial");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
  ros::Time current_time;
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    current_time = ros::Time::now();
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.w = w;
    msg.pose.pose.orientation.z = z;
    /*Publish initialpose just 5 times.*/
    if (x != 0) {
       chatter_pub.publish(msg);
    }
    /*Shutdown node after 5 times*/
    if (count >= 5) {
      ros::shutdown();
    }
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  ros::spin();

  return 0;
}
