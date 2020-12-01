#ifndef TURTLEBOT_MAP_MANAGER_H
#define TURTLEBOT_MAP_MANAGER_H

#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"

namespace exploration {

using geometry_msgs::Point;
using nav_msgs::OccupancyGridConstPtr;

/**
 * @brief Manages raw occupancy grid data.
 */
class MapManager {
public:
  /**
   * @brief Represents the state of a cell in the map.
   */
  enum class CellState {
    /// We can safely move to the cell.
    FREE,
    /// An obstacle is blocking the cell.
    OCCUPIED,
    /// We do not know the state of this cell.
    UNKNOWN,
    /// We were unable to plan a path to here.
    UNREACHABLE,
  };

  /**
   * @brief Convenience structure for representing the location of a grid cell.
   */
  struct CellLocation {
    uint32_t x, y;
  };

  /// Defines a hash for CellLocation.
  struct CellLocationHash {
    size_t operator()(const CellLocation &cell) const {
      return std::hash<uint32_t>{}(cell.x) ^ std::hash<uint32_t>{}(cell.y);
    }
  };

  using CellSet = std::unordered_set<CellLocation, CellLocationHash>;

  /**
   * @brief Updates the current stored map.
   * @param map_message The message containing the new map data.
   */
  void UpdateMap(const OccupancyGridConstPtr &map_message);

  /**
   * @return The width of the map, in cells.
   */
  uint32_t MapWidth() const;
  /**
   * @return The height of the map, in cells.
   */
  uint32_t MapHeight() const;
  /**
   * @brief Gets the state of a particular cell in the map.
   * @param x The x-coordinate of the cell.
   * @param y The y-coordinate of the cell.
   * @return The state of the cell.
   */
  CellState GetCellState(uint32_t x, uint32_t y) const;

  /**
   * @brief Gets the locations of all cells with a particular state.
   * @param state The state of the cells to get.
   * @return A set of all relevant cell locations.
   */
  std::vector<CellLocation> FindAllWithState(CellState state) const;

  /**
   * @brief Gets the locations of all cells in the map that are at most once
   * cell away from this one. (Includes diagonal cells as well.)
   * @param x The x-coordinate of the center cell.
   * @param y The y-coordinate of the center cell.
   * @return A set of all relevant cell locations.
   */
  std::vector<CellLocation> GetAdjacent(uint32_t x, uint32_t y) const;

  /**
   * @brief Uses BFS to find all cells that are contiguous with a specific one.
   * @param start The cell to start searching at.
   * @param[in/out] nodes The cells to include in the search. Note that any
   *    cells added to the component will be removed from this set.
   * @param[out] component Populated with the cells in the final connected
   *    component.
   */
  void FindConnected(const CellLocation &start, CellSet *nodes,
                     CellSet *component) const;

  /**
   * @brief Finds all contiguous nodes that have a particular state.
   * @param state The state that the nodes should have.
   * @return A vector of sets. Each set contains the locations of the cells in
   *    one contiguous blob.
   */
  std::vector<CellSet> FindConnectedWithState(CellState state) const;

  /**
   * @brief Gets the cell that corresponds to a particular location in
   *    map-space.
   * @param point The point in map-space.
   * @return The corresponding cell coordinates.
   */
  CellLocation GetCellAtPoint(const Point &point) const;

  /**
   * @brief Gets the center point of a cell in map-space.
   * @param x The x-coordinate of the cell.
   * @param y The y-coordinate of the cell.
   * @return The center point of the cell.
   */
  Point GetCellCenter(uint32_t x, uint32_t y) const;

  /**
   * @brief Marks a particular cell as unreachable. This designation will
   * persist across map updates.
   * @param x The x-coordinate of the cell.
   * @param y The y-coordinate of the cell.
   */
  void MarkCellUnreachable(uint32_t x, uint32_t y);

private:
  /// Current map that we are using.
  OccupancyGridConstPtr map_{};
  /// Set of cells that we have marked as unreachable.
  CellSet unreachable_{};
};

/**
 * @brief Comparison operator for CellLocation.
 * @param lhs The first location.
 * @param rhs The second location.
 * @return Whether they are equal.
 */
inline bool operator==(const MapManager::CellLocation &lhs,
                       const MapManager::CellLocation &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

} // namespace exploration

#endif // TURTLEBOT_MAP_MANAGER_H
