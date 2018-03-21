#ifndef _COSTMAP_MATH_H_
#define _COSTMAP_MATH_H_

#include <math.h>
#include <algorithm>
#include <vector>
#include <sensor/lidar2d.h>

//TODO replace below several functions
/** @brief Return -1 if x < 0, +1 otherwise. */
inline double sign(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

/** @brief Same as sign(x) but returns 0 if x is 0. */
inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

inline double distance(double x0, double y0, double x1, double y1)
{
  return hypot(x1 - x0, y1 - y0);
}

double
distanceToLine(double pX, double pY, double x0, double y0, double x1,
               double y1);
///not used
bool
intersects(std::vector< Point2D >& polygon, float testx,
           float testy);

bool
intersects(std::vector< Point2D >& polygon1,
           std::vector< Point2D >& polygon2);

#endif  // COSTMAP_2D_COSTMAP_MATH_H_
