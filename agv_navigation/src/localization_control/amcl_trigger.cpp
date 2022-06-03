#include <agv_navigation/localization_control/amcl_trigger.h>


AmclTrigger::AmclTrigger(std::string car_state_topic, int rate)
{
    ros::NodeHandle nh;
    
    amcl_client_ = nh.serviceClient<std_srvs::Empty>("/request_nomotion_update");
    ros::service::waitForService ("/request_nomotion_update", 100);
    rate_ = rate;

    sub_car_state = nh.subscribe(car_state_topic,10, &AmclTrigger::carStateCallback, this); 



    ros::Rate loop_rate(rate_);
    while (ros::ok()) {

        if(car_state_msg.car_is_stop)
        {
            if(!amcl_client_.call(s)){
            ROS_ERROR("Amcl  request_nomotion_update Error");
            }
        }


        ros::spinOnce();
        loop_rate.sleep();
    }

}

AmclTrigger::~AmclTrigger()
{
}

void AmclTrigger::carStateCallback(const agv_navigation::CarStateMsgs _car_state)
{
    car_state_msg = _car_state;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "amcl_trigger_node");
    
    AmclTrigger imu_control = AmclTrigger("/car_state",2); 
    
    return 0;
}