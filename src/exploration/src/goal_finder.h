#ifndef TURTLEBOT_GOAL_FINDER_H
#define TURTLEBOT_GOAL_FINDER_H

#include <vector>

#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"

#include "map_manager.h"

namespace exploration {

using geometry_msgs::Pose;
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
   * @brief Finds the unoccupied cell that is the closest to our current
   * location.
   * @param current_pose Our current location.
   * @return The location of the closest unoccupied cell.
   */
  MapManager::CellLocation FindNearestUnexplored(const Pose &current_pose);

  /**
   * @brief Checks if a cell has any neighbors that are free.
   * @param x The x-coordinate of the cell.
   * @param y The y-coordinate of the cell.
   * @return True iff any neighboring cells are free.
   */
  bool HasFreeNeighbors(uint32_t x, uint32_t y) const;

  std::vector<MapManager::CellSet> FindConnectedEdges();

  /// The internal map to use for planning.
  MapManager *map_;
};

} // namespace exploration

#endif // TURTLEBOT_GOAL_FINDER_H
