#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
class LaserScanToPointCloud{
public:
  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  ros::Subscriber laser_sub_;
  ros::Publisher scan_pub_;
 LaserScanToPointCloud(ros::NodeHandle n)
 {
  // std::cout << "LaserScanToPointCloud Girdiiiiiiiiiii";
    n_ = n;
    laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("scan",10,&LaserScanToPointCloud::scanCallback,this);
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/tf_cloud", 1);
 }
  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    //std::cout << "scanCallBack Girdiiiiiiiiiii";
    if(!listener_.waitForTransform(
            scan_in->header.frame_id,
            "/map",
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
            ros::Duration(2.0))){
        return;
      }
      sensor_msgs::PointCloud cloud;
      projector_.transformLaserScanToPointCloud("/map",*scan_in,
              cloud,listener_);
      scan_pub_.publish(cloud);
  }
};
int main(int argc, char** argv)
{
  //std::cout << "main Girdiiiiiiiiiii";
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  ros::spin();
  return 0;
}

