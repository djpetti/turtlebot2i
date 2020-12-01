#ifndef TURTLEBOT_UTILS_H
#define TURTLEBOT_UTILS_H

#include "geometry_msgs/Point.h"

namespace exploration {

using geometry_msgs::Point;

/**
 * @brief Computes the Euclidean distance between two points.
 * @param point1 The first point.
 * @param point2 The second point.
 * @return The distance between the points.
 */
double EuclideanDistance(const Point &point1, const Point &point2);

}  // namespace exploration

#endif // TURTLEBOT_UTILS_H
