#ifndef WAREHOUSE_LAYER_H
#define WAREHOUSE_LAYER_H

#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <float.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <limits.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <agv_navigation/warehouse_costs.h>

// Here, we'll create a new layer to store highway and restricted area costs.
namespace warehouse
{

class WarehouseLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{

public:
  WarehouseLayer();

  virtual void onInitialize();

  // This method is called by the LayeredCostmap to poll this plugin as to how much of the
  // costmap it needs to update. Each layer can increase the size of this bounds.
  // Here we update the bounds of our warehouse costmap. It should be maximum of the
  // previous frame's costmap and the newly created polygons (if any).
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);

  
  // This method updates the master costmap that this layer is attached to (in this case
  // this is global costmap). In this method, private member costmap_'s costs are
  // transferred to global costmap on places where global costmap's costs are FREE_SPACE.
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                           int max_i, int max_j);

  virtual void matchSize();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  // This callback shouldn't have a complex operation, it simply deep copies the received
  // polygons topic, to the private member polygons_.
  void polygonPublisherCB(const jsk_recognition_msgs::PolygonArray &polygons);

  // ROS subscribers, polygons that were created by the subscribed topics and configure
  // server.
  ros::Subscriber sub_;

  // List of polygons should be received as a private member of this class. This polygon
  // will be received from a rosbag file (if we'd like to have the same costs all the time)
  // or it should be received from a live topic.
  jsk_recognition_msgs::PolygonArray polygons_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

};

}


#endif