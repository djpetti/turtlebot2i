#ifndef TURTLEBOT_MAP_MANAGER_H
#define TURTLEBOT_MAP_MANAGER_H

#include <cstdint>
#include <vector>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"

namespace exploration {

using nav_msgs::OccupancyGridConstPtr;
using geometry_msgs::Point;

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
  };

  /**
   * @brief Convenience structure for representing the location of a grid cell.
   */
  struct CellLocation {
    uint32_t x, y;
  };

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
   * @brief Gets the cell that corresponds to a particular location in
   *    map-space.
   * @param point The point in map-space.
   * @return The corresponding cell coordinates.
   */
  CellLocation GetCellAtPoint(const Point& point) const;

  /**
   * @brief Gets the center point of a cell in map-space.
   * @param x The x-coordinate of the cell.
   * @param y The y-coordinate of the cell.
   * @return The center point of the cell.
   */
  Point GetCellCenter(uint32_t x, uint32_t y) const;

private:
  /// Current map that we are using.
  OccupancyGridConstPtr map_{};
};

} // namespace exploration

#endif // TURTLEBOT_MAP_MANAGER_H
