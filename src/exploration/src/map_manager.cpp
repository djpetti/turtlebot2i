#include "map_manager.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <queue>

#include "costmap_2d/cost_values.h"
#include "ros/ros.h"

namespace exploration {
namespace {

/**
 * @brief Costmap threshold value at which to consider a cell occupied, between
 * 0 and 100.
 */
constexpr uint8_t kOccupancyThreshold = 100;
/// Maximum number of cycles to calculate paths for.
constexpr uint32_t kMaxPathPlanCycles = 1000000;

/**
 * @brief Determines how much of a shift there has been in the origin between
 *  two maps.
 * @param previous_map The earlier map.
 * @param new_map The current map.
 * @param[out] shift_x Set to the x-value of the shift in grid cells.
 * @param[out] shift_y Set to the y-value of the shift in grid cells.
 */
void FindMapShift(const OccupancyGridConstPtr &previous_map,
                  const OccupancyGridConstPtr &new_map, int *shift_x,
                  int *shift_y) {
  // Implicit assumption here is that the resolution remains constant.
  ROS_FATAL_COND(std::abs(new_map->info.resolution -
                          previous_map->info.resolution) > 0.0001,
                 "Resolution for all maps must remain constant.");

  // Find the displacement in meters.
  const auto &kNewOrigin = new_map->info.origin.position;
  const auto &kPreviousOrigin = previous_map->info.origin.position;
  const double kOriginShiftX = kNewOrigin.x - kPreviousOrigin.x;
  const double kOriginShiftY = kNewOrigin.y - kPreviousOrigin.y;

  // Convert to cells.
  const double kMPerCell = new_map->info.resolution;
  *shift_x = static_cast<int>(std::round(kOriginShiftX / kMPerCell));
  *shift_y = static_cast<int>(std::round(kOriginShiftY / kMPerCell));

  ROS_INFO_STREAM("Map has shifted by " << *shift_x << ", " << *shift_y
                                        << " cells.");
}

} // namespace

void MapManager::UpdateMap(const OccupancyGridConstPtr &map_message) {
  ROS_DEBUG_STREAM("Updating stored map.");

  // Update unreachable cells if necessary.
  if (map_ != nullptr) {
    UpdateUnreachableLocations(map_message);
  }

  map_ = map_message;

  const auto kOrigin = map_->info.origin.position;
  ROS_DEBUG_STREAM("Map origin: " << kOrigin.x << ", " << kOrigin.y);
}

MapManager::CellState MapManager::GetCellState(uint32_t x, uint32_t y) const {
  ROS_FATAL_COND(map_ == nullptr, "Map is not initialized.");
  ROS_FATAL_COND(y >= MapHeight(), "Specified y-coordinate is out-of-bounds.");
  ROS_FATAL_COND(x >= MapWidth(), "Specified x-coordinate is out-of-bounds.");

  const auto raw_map_value = map_->data[y * MapWidth() + x];

  if (unreachable_.find({x, y}) != unreachable_.end()) {
    // This cell is unreachable.
    return CellState::UNREACHABLE;
  }

  return raw_map_value >= kOccupancyThreshold ? CellState::OCCUPIED
         : raw_map_value == -1                ? CellState::UNKNOWN
                                              : CellState::FREE;
}

uint32_t MapManager::MapWidth() const {
  ROS_FATAL_COND(map_ == nullptr, "Map is not initialized.");
  return map_->info.width;
}

uint32_t MapManager::MapHeight() const {
  ROS_FATAL_COND(map_ == nullptr, "Map is not initialized.");
  return map_->info.height;
}

std::vector<MapManager::CellLocation>
MapManager::FindAllWithState(MapManager::CellState state) const {
  std::vector<CellLocation> locations;

  // Iterate through all the cells.
  for (uint32_t x = 0; x < MapWidth(); ++x) {
    for (uint32_t y = 0; y < MapHeight(); ++y) {
      if (GetCellState(x, y) == state) {
        locations.push_back({x, y});
      }
    }
  }

  return locations;
}

MapManager::CellLocation MapManager::GetCellAtPoint(const Point &point) const {
  ROS_FATAL_COND(map_ == nullptr, "Map is not initialized.");

  // Re-frame relative to the origin.
  const double kXOffset = point.x - map_->info.origin.position.x;
  const double kYOffset = point.y - map_->info.origin.position.y;

  const auto kMPerCell = map_->info.resolution;
  const double kCellX = kXOffset / kMPerCell;
  const double kCellY = kYOffset / kMPerCell;

  // Since cells are indexed from 0, truncation is indeed the right behavior.
  return {static_cast<uint32_t>(kCellX), static_cast<uint32_t>(kCellY)};
}

Point MapManager::GetCellCenter(uint32_t x, uint32_t y) const {
  ROS_FATAL_COND(map_ == nullptr, "Map is not initialized.");

  const auto kMPerCell = map_->info.resolution;

  // Find the corner point.
  const double kCornerXM = x * kMPerCell;
  const double kCornerYM = y * kMPerCell;

  // Translate to the map frame.
  const double kCornerXMapM = kCornerXM + map_->info.origin.position.x;
  const double kCornerYMapM = kCornerYM + map_->info.origin.position.y;

  // Adjust to get the center.
  Point center;
  center.x = kCornerXMapM + kMPerCell / 2;
  center.y = kCornerYMapM + kMPerCell / 2;
  center.z = 0.0;

  return center;
}
std::vector<MapManager::CellLocation>
MapManager::GetAdjacent(uint32_t x, uint32_t y) const {
  ROS_FATAL_COND(map_ == nullptr, "Map is not initialized.");
  ROS_FATAL_COND(y >= MapHeight(), "Specified y-coordinate is out-of-bounds.");
  ROS_FATAL_COND(x >= MapWidth(), "Specified x-coordinate is out-of-bounds.");

  // Determine the neighborhood of the cell.
  std::vector<uint32_t> x_values{x - 1, x, x + 1};
  std::vector<uint32_t> y_values{y - 1, y, y + 1};

  std::vector<CellLocation> neighbor_locations;
  for (auto search_x : x_values) {
    if (search_x >= MapWidth() || search_x < 0) {
      // X out-of-bounds.
      continue;
    }

    for (auto search_y : y_values) {
      if (search_y >= MapHeight() || search_y < 0) {
        // Y out-of-bounds.
        continue;
      }

      neighbor_locations.push_back({search_x, search_y});
    }
  }

  return neighbor_locations;
}

void MapManager::MarkCellUnreachable(uint32_t x, uint32_t y) {
  ROS_INFO_STREAM("Marking cell at " << x << ", " << y << " as unreachable.");

  unreachable_.insert({x, y});
}

void MapManager::FindConnected(const MapManager::CellLocation &start,
                               MapManager::CellSet *nodes,
                               MapManager::CellSet *component) const {
  std::queue<CellLocation> bfs_queue;
  bfs_queue.push(start);

  component->clear();

  while (!bfs_queue.empty()) {
    const auto &cell = bfs_queue.back();
    bfs_queue.pop();
    // We've seen this cell.
    nodes->erase(cell);
    component->insert(cell);

    // Find unexplored neighbors.
    const auto &neighbors = GetAdjacent(cell.x, cell.y);
    for (const auto &neighbor : neighbors) {
      if (nodes->find(neighbor) != nodes->end()) {
        bfs_queue.push(neighbor);
      }
    }
  }
}

std::vector<MapManager::CellSet>
MapManager::FindConnectedWithState(MapManager::CellState state) const {
  // First, get only the nodes that have this state.
  const auto &nodes_vector = FindAllWithState(state);
  CellSet nodes(nodes_vector.begin(), nodes_vector.end());

  // We basically use BFS to find connected components.
  std::vector<CellSet> components;
  while (!nodes.empty()) {
    CellSet component;
    FindConnected(*nodes.begin(), &nodes, &component);
    components.push_back(component);
  }

  return components;
}

void MapManager::UpdateUnreachableLocations(
    const OccupancyGridConstPtr &new_map) {
  // Determine the shift in the origin between the two maps.
  int shift_x, shift_y;
  FindMapShift(map_, new_map, &shift_x, &shift_y);

  const CellSet kOldUnreachable = unreachable_;
  unreachable_.clear();

  // Shift all the unreachable cells by this amount.
  for (const auto &cell : kOldUnreachable) {
    if (-shift_x > static_cast<int>(cell.x) ||
        -shift_y > static_cast<int>(cell.y)) {
      // This cell is no longer within the map.
      ROS_DEBUG_STREAM("Removing unreachable cell at "
                       << cell.x << ", " << cell.y
                       << " because it is outside the map.");
      continue;
    }

    unreachable_.insert({cell.x + shift_x, cell.y + shift_y});
  }
}

const std::vector<uint8_t> &MapManager::AsCostmap() {
  costmap_.clear();
  const uint32_t kDataLength = MapHeight() * MapWidth();
  costmap_.resize(kDataLength);

  for (uint32_t i = 0; i < kDataLength; ++i) {
    // Note: The "magic" values here are taken from the costmap publisher code.
    switch (map_->data[i]) {
    case -1: {
      // This indicates an unknown location.
      costmap_[i] = costmap_2d::NO_INFORMATION;
      break;
    }
    case 0: {
      // This indicates a free space.
      costmap_[i] = costmap_2d::FREE_SPACE;
      break;
    }
    case 99: {
      // This indicates an inscribed obstacle.
      costmap_[i] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      break;
    }
    case 100: {
      // This indicates a lethal obstacle.
      costmap_[i] = costmap_2d::LETHAL_OBSTACLE;
    }
    default: {
      // Otherwise, just rescale the values directly.
      const double kRescaled = static_cast<double>(map_->data[i]) / 100.0 *
                               costmap_2d::LETHAL_OBSTACLE;
      costmap_[i] = static_cast<uint8_t>(std::round(kRescaled));
    }
    }
  }

  return costmap_;
}

float MapManager::CalculateNavigationCost(
    const MapManager::CellLocation &start,
    const MapManager::CellLocation &goal) {
  // Initialize to the correct grid size if necessary.
  if (MapWidth() != static_cast<uint32_t>(nav_.nx) ||
      MapHeight() != static_cast<uint32_t>(nav_.ny)) {
    ROS_DEBUG_STREAM("Reinitializing NavFn with costmap of size "
                     << MapWidth() << "x" << MapHeight());
    nav_.setNavArr(MapWidth(), MapHeight());
  }

  // Set the parameters.
  const auto &costmap = AsCostmap();
  nav_.setCostmap(costmap.data());
  int start_array[] = {static_cast<int>(start.x), static_cast<int>(start.y)};
  int goal_array[] = {static_cast<int>(goal.x), static_cast<int>(goal.y)};
  nav_.setStart(start_array);
  nav_.setGoal(goal_array);

  nav_.setupNavFn(true);

  // Calculate the potential and path.
  const uint32_t kMaxIterations =
      std::max(MapWidth() * MapHeight() / 20, MapWidth() + MapHeight());
  nav_.propNavFnAstar(kMaxIterations);

  // Find a path.
  if (nav_.calcPath(kMaxPathPlanCycles) <= 0) {
    // We did not find a full path to the start.
    ROS_WARN_STREAM("Did not find a full path from "
                    << start.x << ", " << start.y << " to " << goal.x << ", "
                    << goal.y << ".");
    return std::numeric_limits<float>::infinity();
  }

  // Get the cost of the path.
  return nav_.getLastPathCost();
}

} // namespace exploration