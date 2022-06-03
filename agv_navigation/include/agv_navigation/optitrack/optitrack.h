#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <udp_client_server.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

class OptiTrack
{
private:
    int _rate;
    ros::Publisher pub_odom;
    ros::Publisher pub_pose2d;

    geometry_msgs::Pose2D pose2d;
    nav_msgs::Odometry odom_msg_;

    udp_client_server::udp_server *_udp;

    char data_raw[15];
    std::vector<double> data_split;
public:
    OptiTrack(std::string odom_topic, std::string pose2d_topic, int rate);
    ~OptiTrack();
   void readDataUdp();

};


