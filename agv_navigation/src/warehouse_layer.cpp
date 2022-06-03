#include <agv_navigation/warehouse_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(warehouse::WarehouseLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using warehouse::WAREHOUSE_FREE_SPACE;
using warehouse::HIGHWAY_COST;
using warehouse::RESTRICTED_AREA_COST;

namespace warehouse
{

WarehouseLayer::WarehouseLayer() {}

void WarehouseLayer::onInitialize() 
{
  // Create the warehouse layer node
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  // Other than highway and restricted area, this layer's default cost is free space.
  default_value_ = WAREHOUSE_FREE_SPACE;
  matchSize();

  std::string costmap_topic_name;
  if (ros::param::get("/move_base/global_costmap/warehouse_layer/costmap_topic", costmap_topic_name)) {
    sub_ = nh.subscribe(
      "/" + costmap_topic_name, 10, &WarehouseLayer::polygonPublisherCB, this
    );
  }
  
  // This layer uses the GenericPluginConfig which consists of only a flag called enabled,
  // for easy enabling and disabling of this particular layer. You can create your own
  // custom dynamic_reconfigure configuration and insert it instead.
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType
    cb = boost::bind(&WarehouseLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void WarehouseLayer::matchSize()
{
  // Gets the main costmap that this layer is built on (in this case it will be global
  // costmap).
  Costmap2D* master = layered_costmap_->getCostmap();
  // Resized the current costmap according to the master costmap.
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void WarehouseLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void WarehouseLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                               double* min_x, double* min_y, double* max_x, double* max_y)
{
  
  
  if (!enabled_)
    return;

  double curr_min_x = __DBL_MAX__;
  double curr_max_x = __DBL_MIN__;
  double curr_min_y = __DBL_MAX__;
  double curr_max_y = __DBL_MIN__;
  for (geometry_msgs::PolygonStamped curr_polygon_msg : polygons_.polygons) {
    for (geometry_msgs::Point32 point32 : curr_polygon_msg.polygon.points) {
      double point_x = static_cast<double>(point32.x);
      double point_y = static_cast<double>(point32.y);
      curr_min_x = std::min(curr_min_x, point_x);
      curr_max_x = std::max(curr_max_x, point_x);
      curr_min_y = std::min(curr_min_y, point_y);
      curr_max_y = std::max(curr_max_y, point_y);
    }
  }

  *min_x = std::min(*min_x, curr_min_x);
  *min_y = std::min(*min_y, curr_min_y);
  *max_x = std::max(*max_x, curr_max_x);
  *max_y = std::max(*max_y, curr_max_y);

  // Set the costs of our warehouse costmap
  // Call the callback function if there is any new messages received.
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));

  // Traverse through the polygons_ array, and set the costs accordingly. 
  int i = 0;
  for (geometry_msgs::PolygonStamped curr_polygon_msg : polygons_.polygons) {
    uint32_t curr_label = polygons_.labels[i];
    geometry_msgs::Polygon curr_polygon = curr_polygon_msg.polygon;
    
    // Convert points array to a vector
    std::vector<geometry_msgs::Point> point_vector;
    for (geometry_msgs::Point32 point32 : curr_polygon.points) {
      geometry_msgs::Point point;
      point.x = point32.x;
      point.y = point32.y;
      point.z = point32.z;
      point_vector.push_back(point);
    }

    // 
    int highway_type, restricted_area_type;
    if (ros::param::get("/move_base/global_costmap/warehouse_layer/polygon_types/highway", highway_type) &&
        ros::param::get("/move_base/global_costmap/warehouse_layer/polygon_types/restricted_area", restricted_area_type))
    {
      if (curr_label == highway_type) {
        setConvexPolygonCost(point_vector, HIGHWAY_COST);
      } else if (curr_label == restricted_area_type) {
        setConvexPolygonCost(point_vector, RESTRICTED_AREA_COST);
      }
    }
    i++;
  }
}

void WarehouseLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                              int max_i, int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = getIndex(i, j);
      if (master_grid.getCost(i, j) == FREE_SPACE) {
        master_grid.setCost(i, j, costmap_[index]);
      }
    }
  }

}

void WarehouseLayer::polygonPublisherCB(const jsk_recognition_msgs::PolygonArray &polygons)
{
  polygons_ = polygons;
}
} // end namespace