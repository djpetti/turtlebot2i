#include "map_manager.h"

#include <queue>

#include "ros/ros.h"

namespace exploration {

void MapManager::UpdateMap(const OccupancyGridConstPtr &map_message) {
  ROS_DEBUG_STREAM("Updating stored map.");

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

} // namespace exploration