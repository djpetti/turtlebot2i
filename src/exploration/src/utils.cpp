#include "utils.h"

#include <cmath>

namespace exploration {

double EuclideanDistance(const Point &point1, const Point &point2) {
  return std::sqrt(std::pow(point2.x - point1.x, 2) +
                   std::pow(point2.y - point1.y, 2));
}

}  // namespace exploration