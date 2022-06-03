/*
This code is used for publishing initial position from 
odom topic to /initialpose topic to use in /amcl node. 
*/

#include <agv_navigation/publish_odom.h>

OdomPublisher::OdomPublisher(std::string odom_topic, std::string amcl_pose_topic,
                             std::string odom_motor_topic, int rate) {
    ros::NodeHandle nh;
    
    rate_ = rate;

    sub_amcl_ = nh.subscribe(amcl_pose_topic, 10, &OdomPublisher::amclPoseCallback, this);
    sub_odom_ = nh.subscribe(odom_motor_topic, 10, &OdomPublisher::odomCallback, this);
    
    pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic, 100);

    ros::Rate loop_rate(rate_);
    while (ros::ok()) {
        setOdomMsg();
        pub_.publish(odom_msg_);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void OdomPublisher::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped pose) {
    // pose_.header.stamp = pose.header.stamp;
    // pose_.header.frame_id = pose.header.frame_id;

    // pose_.pose.covariance = pose.pose.covariance;
    
    pose_.pose.pose.position.x = pose.pose.pose.position.x;
    pose_.pose.pose.position.y = pose.pose.pose.position.y;
    pose_.pose.pose.position.z = pose.pose.pose.position.z;

    pose_.pose.pose.orientation.x = pose.pose.pose.orientation.x;
    pose_.pose.pose.orientation.y = pose.pose.pose.orientation.y;
    pose_.pose.pose.orientation.z = pose.pose.pose.orientation.z;
    pose_.pose.pose.orientation.w = pose.pose.pose.orientation.w;
    
}

void OdomPublisher::odomCallback(const nav_msgs::Odometry odom) {
    odom_motor_msg_.header.stamp = odom.header.stamp;
    odom_motor_msg_.header.frame_id = "map";
    odom_motor_msg_.child_frame_id = odom.child_frame_id;

    odom_motor_msg_.pose.covariance = odom.pose.covariance;   
    odom_motor_msg_.twist = odom.twist;
    
}

void OdomPublisher::setOdomMsg() {

    odom_msg_.header = odom_motor_msg_.header;
    odom_msg_.child_frame_id = odom_motor_msg_.child_frame_id;
    
    odom_msg_.pose = pose_.pose;
    odom_msg_.pose.covariance = odom_motor_msg_.pose.covariance;
    odom_msg_.twist = odom_motor_msg_.twist;

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_publisher");
    
    OdomPublisher odom_publisher = OdomPublisher("/odom", "/amcl_pose", "/odom_motor", 20);
    
    return 0;
}

// /*Subscriber for  odom topic*/
// void odomCallback(const nav_msgs::Odometry msg) {
//   x = msg.pose.pose.position.x;
//   y = msg.pose.pose.position.y;
//   w = msg.pose.pose.orientation.w;
//   z = msg.pose.pose.orientation.z;
// }

// int main(int argc, char **argv) {
//   ros::init(argc, argv, "initial");
//   ros::NodeHandle n;
//   ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback);
//   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
//   ros::Time current_time;
//   ros::Rate loop_rate(10);

//   int count = 0;
//   while (ros::ok()) {
//     current_time = ros::Time::now();
//     geometry_msgs::PoseWithCovarianceStamped msg;
//     msg.header.stamp = current_time;
//     msg.header.frame_id = "map";
//     msg.pose.pose.position.x = x;
//     msg.pose.pose.position.y = y;
//     msg.pose.pose.orientation.w = w;
//     msg.pose.pose.orientation.z = z;
//     /*Publish initialpose just 5 times.*/
//     if (x != 0) {
//        chatter_pub.publish(msg);
//     }
//     /*Shutdown node after 5 times*/
//     if (count >= 5) {
//       ros::shutdown();
//     }
//     ros::spinOnce();
//     loop_rate.sleep();
//     count++;
//   }

//   ros::spin();

//   return 0;
// }
