#include "goal_finder.h"

#include <algorithm>
#include <cmath>

#include "ros/ros.h"

namespace exploration {
namespace {

/**
 * @brief If we are too close to an unexplored cell, we're probably
 * right on top of it and won't be able to survey it. Consequently,
 * we enforce a minimum distance away for our goal.
 */
constexpr double kMinUnexploredDistance = 1.0;

/**
 * @brief Computes the Euclidean distance between two points.
 * @param point1 The first point.
 * @param point2 The second point.
 * @return The distance between the points.
 */
double EuclideanDistance(const Point &point1, const Point &point2) {
  return std::sqrt(std::pow(point2.x - point1.x, 2) +
                   std::pow(point2.y - point1.y, 2));
}

} // namespace

GoalFinder::GoalFinder(MapManager *manager) : map_(manager) {}

MapManager::CellLocation
GoalFinder::FindNearestUnexplored(const geometry_msgs::Pose &current_pose) {
  // Find all unexplored nodes.
  const auto unexplored =
      map_->FindAllWithState(MapManager::CellState::UNKNOWN);
  ROS_DEBUG_STREAM("Have " << unexplored.size() << " unexplored cells.");

  // Otherwise, find the one that's closest to us.
  double min_distance = std::numeric_limits<double>::infinity();
  // If we've explored the whole map, we want to stay where we are.
  MapManager::CellLocation closest_cell =
      map_->GetCellAtPoint(current_pose.position);
  for (const auto &cell : unexplored) {
    const auto kCellCenter = map_->GetCellCenter(cell.x, cell.y);
    const auto kDistance =
        EuclideanDistance(kCellCenter, current_pose.position);

    if (kDistance < kMinUnexploredDistance) {
      // We won't be able to survey this cell.
      continue;
    }

    if (kDistance < min_distance) {
      // New closest cell found.
      min_distance = kDistance;
      closest_cell = cell;
    }
  }

  return closest_cell;
}

Pose GoalFinder::FindNewGoal(const nav_msgs::OccupancyGridConstPtr &new_map,
                             const Pose &current_pose) {
  // Update the map.
  map_->UpdateMap(new_map);

  // Locate the closest unexplored cell.
  const auto kGoalCell = FindNearestUnexplored(current_pose);

  // Use that location as our goal.
  Pose goal = current_pose;
  goal.position = map_->GetCellCenter(kGoalCell.x, kGoalCell.y);
  ROS_ERROR_STREAM("Setting new goal to [" << goal.position.x << ", "
                                           << goal.position.y << "].");

  return goal;
}

} // namespace exploration
