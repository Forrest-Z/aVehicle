#include <agv_navigation/optitrack/optitrack.h>



OptiTrack::OptiTrack(std::string odom_topic, std::string pose2d_topic, int rate)
{
    _udp = new udp_client_server::udp_server("192.168.10.25",8080);

    ros::NodeHandle nh;
    _rate = rate;

    pub_odom = nh.advertise<nav_msgs::Odometry>(odom_topic, 100);
    pub_pose2d = nh.advertise<geometry_msgs::Pose2D>(pose2d_topic, 100);

    ros::Rate loop_rate(_rate);
    while (ros::ok()) {
        readDataUdp();
        pub_odom.publish(odom_msg_);
        pub_pose2d.publish(pose2d);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

OptiTrack::~OptiTrack()
{
}

void OptiTrack::readDataUdp()
{

    if(_udp->timed_recv(data_raw,sizeof(char)*128,250) == -1)
    {
        ROS_WARN_STREAM("Warn Optitrack Udp ERR:" << errno <<"\n" );
    }
    ROS_INFO_STREAM("recieved data from Udp : " << data_raw <<"\n" );
    
    std::stringstream ss(data_raw);
    int  i  = 0;
    double data_[10] = {};
    while (ss.good()  && i < 10)
    {
        std::string substr;
        getline(ss, substr, '|');
        data_[i]= std::stod(substr);
        i++;
    } 
    /*double a;
    char b;
    while (ss>>a)
    {
        ss>>b;
        data_split.push_back(a);
    }
    */
    //for (auto i = data_split.begin(); i != data_split.end(); ++i)
        //ROS_INFO_STREAM( *i << " \n");
    odom_msg_.header.stamp = ros::Time::now();    
    odom_msg_.pose.pose.position.x      = data_[0];
    odom_msg_.pose.pose.position.z      = data_[1];
    odom_msg_.pose.pose.position.y      = data_[2];
    odom_msg_.pose.pose.orientation.w   = data_[3];
    odom_msg_.pose.pose.orientation.x   = data_[4];
    odom_msg_.pose.pose.orientation.z   = data_[5];
    odom_msg_.pose.pose.orientation.y   = data_[6];

    odom_msg_.twist.twist.linear.x      = data_[7];
    odom_msg_.twist.twist.linear.y      = data_[8];
    odom_msg_.twist.twist.angular.z     = data_[9];

    //ROS_INFO_STREAM("data split length: " << data_size() << "\n");

    tf::Quaternion q(
        odom_msg_.pose.pose.orientation.x,
        odom_msg_.pose.pose.orientation.y,
        odom_msg_.pose.pose.orientation.z,
        odom_msg_.pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose2d.theta = yaw;
    pose2d.x = data_[0];
    pose2d.y = data_[2];

}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "optitrack");
    OptiTrack optitrack = OptiTrack("/optitrack/odom","/optitrack/pose2d",50);
    return 0;
}
