#ifndef TURTLEBOT_GOAL_FINDER_H
#define TURTLEBOT_GOAL_FINDER_H

#include <vector>

#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"

#include "map_manager.h"

namespace exploration {

using geometry_msgs::Pose;
using nav_msgs::OccupancyGrid;
using nav_msgs::OccupancyGridConstPtr;

/**
 * @brief Responsible for choosing the next goal to navigate to during
 *  the exploration procedure.
 */
class GoalFinder {
public:
  /**
   * @param manager The MapManager to use internally.
   */
  explicit GoalFinder(MapManager *manager);

  /**
   * @brief Determines a new exploration goal when given a current map.
   * @param new_map The map to use for planning.
   * @param current_pose Our current pose.
   * @return The new goal it has selected.
   */
  Pose FindNewGoal(const OccupancyGridConstPtr &new_map,
                   const Pose &current_pose);

  /**
   * @brief Marks a particular goal as unreachable. This will prevent it
   * from ever suggesting we move there again.
   * @param goal The goal that is unreachable.
   */
  void MarkGoalUnreachable(const Pose &goal);

private:
  /**
   * @brief Finds the unexplored edge for which the cost of the path is
   *    lowest.
   * @param current_pose Our current location.
   * @return The location of the closest unoccupied cell.
   */
  MapManager::CellLocation FindLowestCostUnexplored(const Pose &current_pose);

  /**
   * @brief Checks if a cell has any neighbors that are free.
   * @param x The x-coordinate of the cell.
   * @param y The y-coordinate of the cell.
   * @return True iff any neighboring cells are free.
   */
  bool HasFreeNeighbors(uint32_t x, uint32_t y) const;

  /**
   * @brief Updates the stored set of connected edge nodes. An edge node is
   * an unexplored node that borders on a free one.
   */
  void UpdateUnexploredEdges();

  /**
   * @brief Given a blob, it finds the cell within that blob that is nearest
   *    to the center.
   * @param blob The blob to find the center of.
   * @return The cell within the blob that is nearest to the center.
   */
  MapManager::CellLocation
  FindMostCenteredCellInBlob(const MapManager::CellSet &blob);

  /// The internal map to use for planning.
  MapManager *map_;
  /// The internal costmap to use for planning.
  OccupancyGrid costmap_{};
  /// Set of connected edge nodes in the map.
  std::vector<MapManager::CellSet> unexplored_edges_;
};

} // namespace exploration

#endif // TURTLEBOT_GOAL_FINDER_H
