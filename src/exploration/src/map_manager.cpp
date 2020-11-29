#include "map_manager.h"

#include "ros/ros.h"

namespace exploration {

void MapManager::UpdateMap(const OccupancyGridConstPtr &map_message) {
  ROS_DEBUG_STREAM("Updating stored map.");

  map_ = map_message;
}

MapManager::CellState MapManager::GetCellState(uint32_t x, uint32_t y) const {
  ROS_FATAL_COND(map_ == nullptr, "Map is not initialized.");
  ROS_FATAL_COND(y >= MapHeight(), "Specified y-coordinate is out-of-bounds.");
  ROS_FATAL_COND(x >= MapWidth(), "Specified x-coordinate is out-of-bounds.");

  const auto raw_map_value = map_->data[y * MapHeight() + x];

  // If there is a more than 50% probability that the space is occupied, we
  // mark it as such.
  return raw_map_value > 50    ? CellState::OCCUPIED
         : raw_map_value == -1 ? CellState::UNKNOWN
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

  const auto kMPerCell = map_->info.resolution;
  const double kCellX = point.x / kMPerCell;
  const double kCellY = point.y / kMPerCell;

  // Since cells are indexed from 0, truncation is indeed the right behavior.
  return {static_cast<uint32_t>(kCellX), static_cast<uint32_t>(kCellY)};
}

Point MapManager::GetCellCenter(uint32_t x, uint32_t y) const {
  ROS_FATAL_COND(map_ == nullptr, "Map is not initialized.");

  const auto kMPerCell = map_->info.resolution;

  // Find the corner point.
  const double kCornerXM = x * kMPerCell;
  const double kCornerYM = y * kMPerCell;

  // Adjust to get the center.
  Point center;
  center.x = kCornerXM + kMPerCell / 2;
  center.y = kCornerYM + kMPerCell / 2;
  center.z = 0.0;

  return center;
}

} // namespace exploration