#ifndef PUBLISH_ODOM_H
#define PUBLISH_ODOM_H

/*
This code is used for publish odom topic by subscribing amcl_pose. 
*/

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "ros/console.h"
#include "ros/ros.h"
#include <sstream>


class OdomPublisher
{

public:

    OdomPublisher(std::string odom_topic, std::string amcl_pose_topic, std::string odom_motor_topic, int rate);

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped pose);
    void odomCallback(const nav_msgs::Odometry odom);

private:

    int rate_;

    ros::Subscriber sub_amcl_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_;
    std::string amcl_pose_topic_, odom_topic_;

    nav_msgs::Odometry odom_motor_msg_;
    nav_msgs::Odometry odom_msg_;
    geometry_msgs::PoseWithCovarianceStamped pose_;

    void setOdomMsg();

};

#endif