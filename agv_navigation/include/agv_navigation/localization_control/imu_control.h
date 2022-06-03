#ifndef IMU_CONTROL_H
#define IMU_CONTROL_H

#include "ros/console.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <agv_navigation/CarStateMsgs.h>
#include "sensor_msgs/Imu.h"

class ImuControl
{
private:
    std::uint8_t rate_;

    ros::Subscriber sub_imu_data_raw;
    ros::Subscriber sub_car_state;
    ros::Publisher pub_;
    agv_navigation::CarStateMsgs car_state_msg;
    sensor_msgs::Imu imu_raw_msg_;
    sensor_msgs::Imu imu_updated_msg_;

    
public:
    ImuControl(std::string imu_topic, std::string car_state_topic, 
                std::string publish_topic_name, int rate);
    ~ImuControl();

    void imuRawDataCallback(const sensor_msgs::Imu _imu);
    void carStateCallback(const agv_navigation::CarStateMsgs _car_state);
};

#endif