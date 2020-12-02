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
constexpr uint8_t kMinUnexploredEdgeSize = 15;

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
  // Get the correct angle looking back.
  const double kLookBackAngle = kStraightAngle + M_PI;
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

void GoalFinder::UpdateUnexploredEdges() {
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
  unexplored_edges_.clear();
  while (!edge_nodes.empty()) {
    MapManager::CellSet component;
    map_->FindConnected(*edge_nodes.begin(), &edge_nodes, &component);
    unexplored_edges_.push_back(component);
  }
}

MapManager::CellLocation
GoalFinder::FindMostCenteredCellInBlob(const MapManager::CellSet &blob) {
  // Find the blob center initially.
  const auto &kCenterCell = BlobCenter(blob);
  const auto &kCenterPos = map_->GetCellCenter(kCenterCell.x, kCenterCell.y);

  // Find the cell within the blob that is closest to the center.
  MapManager::CellLocation closest_cell{0, 0};
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto &cell : blob) {
    const auto &kCellPos = map_->GetCellCenter(cell.x, cell.y);

    const double kDistance = EuclideanDistance(kCellPos, kCenterPos);
    if (kDistance < min_distance) {
      closest_cell = cell;
      min_distance = kDistance;
    }
  }

  return closest_cell;
}

MapManager::CellLocation
GoalFinder::FindNearestUnexplored(const geometry_msgs::Pose &current_pose) {
  // Find the edge that's closest to us.
  double min_distance = std::numeric_limits<double>::infinity();
  // If we've explored the whole map, we want to stay where we are.
  MapManager::CellLocation closest_cell =
      map_->GetCellAtPoint(current_pose.position);
  for (const auto &edge : unexplored_edges_) {
    if (edge.size() < kMinUnexploredEdgeSize) {
      // This edge is probably too narrow for us to fit through.
      continue;
    }

    // Find the cell in the blob that's nearest to the center.
    const auto &kCenterCell = FindMostCenteredCellInBlob(edge);

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
  // Update the set of connected edges.
  UpdateUnexploredEdges();
  ROS_DEBUG_STREAM("Have " << unexplored_edges_.size() << " unexplored edges.");

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

  // Find the edge blob that this cell is part of.
  const MapManager::CellSet *unreachable_edge = nullptr;
  for (const auto &edge : unexplored_edges_) {
    if (edge.find({cell.x, cell.y}) != edge.end()) {
      unreachable_edge = &edge;
      break;
    }
  }
  if (unreachable_edge == nullptr) {
    // We must have updated the map before running this, and the corresponding
    // edge has disappeared.
    ROS_WARN_STREAM("No corresponding edge for unreachable cell "
                    << cell.x << ", " << cell.y << ".");
    return;
  }

  // Mark the entire blob as unreachable.
  for (const auto &unreachable_cell : *unreachable_edge) {
    map_->MarkCellUnreachable(unreachable_cell.x, unreachable_cell.y);
  }
}

} // namespace exploration
