#ifndef CAR_STATE_H
#define CAR_STATE_H

#include "ros/console.h"
#include "ros/ros.h"
#include <agv_navigation/CarStateMsgs.h>
#include <agv_bringup/battery.h>
#include "nav_msgs/Odometry.h"

class CarState
{
private:
    int rate_;
    ros::Subscriber sub_battery;
    ros::Subscriber sub_odom_motor;

    ros::Publisher pub_;
    agv_navigation::CarStateMsgs car_state_msg;

public:
    CarState(std::string motor_control_topic, std::string battery_topic,
            std::string publish_topic_name, int rate);
    ~CarState();


    void batteryCallback(const agv_bringup::battery battery_data);
    void odomMotorCallback(const nav_msgs::Odometry odom_motor);
};


#endif