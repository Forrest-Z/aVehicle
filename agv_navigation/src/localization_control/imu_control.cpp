#include <agv_navigation/localization_control/imu_control.h>


ImuControl::ImuControl(std::string imu_topic, std::string car_state_topic, 
                        std::string publish_topic_name, int rate)
{
    ros::NodeHandle nh;
    
    rate_ = rate;

    sub_imu_data_raw = nh.subscribe(imu_topic,10,&ImuControl::imuRawDataCallback, this);
    sub_car_state = nh.subscribe(car_state_topic,10,&ImuControl::carStateCallback, this); 

    pub_ = nh.advertise<sensor_msgs::Imu>(publish_topic_name, 100);

    ros::Rate loop_rate(rate_);
    while (ros::ok()) {
        //checkCarState();
        //updateImuData();

        pub_.publish(imu_updated_msg_);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

ImuControl::~ImuControl()
{
}

void ImuControl::imuRawDataCallback(const sensor_msgs::Imu _imu)
{
    imu_updated_msg_ = _imu;

    if(car_state_msg.car_is_stop)
    {
        //ROS_INFO_STREAM("Car is stop \n");
        imu_updated_msg_.angular_velocity.z = 0.0;
    }
   
}

void ImuControl::carStateCallback(const agv_navigation::CarStateMsgs _car_state)
{
    car_state_msg = _car_state;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_yaw_rate_control");
    
    ImuControl imu_control = ImuControl("/imu/data_raw","/car_state","/imu/data",50); 
    
    return 0;
}