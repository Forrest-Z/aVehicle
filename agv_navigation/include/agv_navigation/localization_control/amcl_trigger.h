#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <agv_navigation/CarStateMsgs.h>
#include <std_srvs/Empty.h>

class AmclTrigger
{
private:
    std::uint8_t rate_;
    ros::ServiceClient amcl_client_;
    std_srvs::Empty s;

    ros::Subscriber sub_car_state;
    agv_navigation::CarStateMsgs car_state_msg;
    
public:
    AmclTrigger(std::string car_state_topic, int rate);
    ~AmclTrigger();

    void carStateCallback(const agv_navigation::CarStateMsgs _car_state);
};
