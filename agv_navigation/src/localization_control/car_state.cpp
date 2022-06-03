#include <agv_navigation/localization_control/car_state.h>


CarState::CarState(std::string motor_control_topic, std::string battery_topic,
            std::string publish_topic_name, int rate )
{ 
    ros::NodeHandle nh;
    
    rate_ = rate;
    
    sub_odom_motor = nh.subscribe(motor_control_topic, 10, &CarState::odomMotorCallback, this);
    sub_battery = nh.subscribe(battery_topic, 10, &CarState::batteryCallback, this);

    pub_ = nh.advertise<agv_navigation::CarStateMsgs>(publish_topic_name, 100);

    ros::Rate loop_rate(rate_);
    while (ros::ok()) {

        pub_.publish(car_state_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

}

CarState::~CarState()
{ }

void CarState::odomMotorCallback(const nav_msgs::Odometry odom_motor)
{
    if(abs(odom_motor.twist.twist.linear.x) < 0.0005 && abs(odom_motor.twist.twist.angular.z) < 0.0005)
    {
        car_state_msg.car_is_stop = true;
        ROS_INFO_STREAM("car_is_stop = \n");
    }
    else
    {
        car_state_msg.car_is_stop = false;
    }
}

void CarState::batteryCallback(const agv_bringup::battery battery_data)
{

    switch (battery_data.state)
    {
    case 0:
        car_state_msg.battery_is_charhing = false;
        break;
    case 1:
        car_state_msg.battery_is_charhing = true;
        break;
    default:
        break;
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "car_state_control");
    CarState car_state = CarState("/odom_motor", "/battery", "/car_state", 30); 

    return 0;
}
