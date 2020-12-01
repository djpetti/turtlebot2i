#include "goal_finder.h"

#include <algorithm>
#include <cmath>

#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "utils.h"

namespace exploration {
namespace {

/**
 * @brief If we are too close to an unexplored cell, we're probably
 * right on top of it and won't be able to survey it. Consequently,
 * we enforce a minimum distance away for our goal.
 */
constexpr double kMinUnexploredDistance = 1.0;
/**
 * @brief We look for unexplored cells that are members of an edge blob with at
 * least this size. Otherwise, they're likely to either be noise, or we won't
 * be able to fit through the opening.
 */
constexpr uint8_t kMinUnexploredEdgeSize = 10;

/**
 * @brief Gets the cell that's closest to the center of a blob.
 * @param blob The blob of cells.
 * @return The location of cell nearest to the center.
 */
MapManager::CellLocation BlobCenter(const MapManager::CellSet &blob) {
  double total_x = 0.0, total_y = 0.0;
  for (const auto &cell : blob) {
    total_x += cell.x;
    total_y += cell.y;
  }

  const uint32_t kAverageX = std::round(total_x / blob.size());
  const uint32_t kAverageY = std::round(total_y / blob.size());
  return {kAverageX, kAverageY};
}

tf2::Quaternion CalculateGoalOrientation(const Point &start,
                                         const Point &goal) {
  // Find the angle between.
  const double kStraightAngle = std::atan2(goal.y - start.y, goal.x - start.x);
  // Subtract from 180 degrees so we get the correct angle looking back.
  const double kLookBackAngle = M_PI - kStraightAngle;
  ROS_DEBUG_STREAM("Goal yaw is " << kLookBackAngle << ".");

  // Convert to a quaternion.
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, kLookBackAngle);
  return quaternion;
}

} // namespace

GoalFinder::GoalFinder(MapManager *manager) : map_(manager) {}

bool GoalFinder::HasFreeNeighbors(uint32_t x, uint32_t y) const {
  // Find all the neighbors first.
  const auto &neighbors = map_->GetAdjacent(x, y);

  // See if any are free.
  return std::any_of(neighbors.begin(), neighbors.end(),
                     [&](const MapManager::CellLocation &cell) {
                       return map_->GetCellState(cell.x, cell.y) ==
                              MapManager::CellState::FREE;
                     });
}

std::vector<MapManager::CellSet> GoalFinder::FindConnectedEdges() {
  // Find all unknown nodes.
  const auto &unknown_nodes =
      map_->FindAllWithState(MapManager::CellState::UNKNOWN);

  // Filter to only those that border on a free region.
  MapManager::CellSet edge_nodes;
  for (const auto &node : unknown_nodes) {
    if (HasFreeNeighbors(node.x, node.y)) {
      edge_nodes.insert(node);
    }
  }

  // Find all connected components among these nodes.
  std::vector<MapManager::CellSet> components;
  while (!edge_nodes.empty()) {
    MapManager::CellSet component;
    map_->FindConnected(*edge_nodes.begin(), &edge_nodes, &component);
    components.push_back(component);
  }

  return components;
}

MapManager::CellLocation
GoalFinder::FindNearestUnexplored(const geometry_msgs::Pose &current_pose) {
  // Find unexplored edges on the map.
  const auto &unexplored_edges = FindConnectedEdges();
  ROS_DEBUG_STREAM("Have " << unexplored_edges.size() << " unexplored edges.");

  // Find the edge that's closest to us.
  double min_distance = std::numeric_limits<double>::infinity();
  // If we've explored the whole map, we want to stay where we are.
  MapManager::CellLocation closest_cell =
      map_->GetCellAtPoint(current_pose.position);
  for (const auto &edge : unexplored_edges) {
    if (edge.size() < kMinUnexploredEdgeSize) {
      // This edge is probably too narrow for us to fit through.
      continue;
    }

    // Find the center cell in the blob.
    const auto &kCenterCell = BlobCenter(edge);

    // Choose the blob that is the minimum distance away from our current
    // position.
    const auto &kCellInMapFrame =
        map_->GetCellCenter(kCenterCell.x, kCenterCell.y);
    const auto kDistance =
        EuclideanDistance(kCellInMapFrame, current_pose.position);

    if (kDistance < kMinUnexploredDistance) {
      // We won't be able to survey this cell.
      continue;
    }

    if (kDistance < min_distance) {
      // New closest cell found.
      min_distance = kDistance;
      closest_cell = kCenterCell;
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
  const auto kGoalPos = map_->GetCellCenter(kGoalCell.x, kGoalCell.y);
  const auto kGoalOrientation =
      CalculateGoalOrientation(current_pose.position, kGoalPos);

  // Use that location as our goal.
  Pose goal;
  goal.position = kGoalPos;
  goal.orientation = tf2::toMsg(kGoalOrientation);
  ROS_ERROR_STREAM("Setting new goal to ["
                   << goal.position.x << ", " << goal.position.y << "] and ["
                   << kGoalOrientation.x() << ", " << kGoalOrientation.y()
                   << ", " << kGoalOrientation.z() << ", "
                   << kGoalOrientation.w() << "].");

  return goal;
}

void GoalFinder::MarkGoalUnreachable(const Pose &goal) {
  // Find the nearest cell.
  const auto &cell = map_->GetCellAtPoint(goal.position);
  // Mark as unreachable.
  map_->MarkCellUnreachable(cell.x, cell.y);
}

} // namespace exploration
