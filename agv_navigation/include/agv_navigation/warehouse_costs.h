#ifndef WAREHOUSE_COSTS_H
#define WAREHOUSE_COSTS_H

// #include <warehouse_costs.h>

using costmap_2d::LETHAL_OBSTACLE;
/** Provides a mapping for often used cost values */
namespace warehouse
{
  // Change the free space on the master_grid to this value, so that highways will be known
  static const unsigned char WAREHOUSE_FREE_SPACE = 100;
  static const unsigned char HIGHWAY_COST = 0;
  static const unsigned char RESTRICTED_AREA_COST = LETHAL_OBSTACLE;
}
#endif  // WAREHOUSE_COSTS_H
