#include "OrientationFilter.h"
#include <Geometry/Angles.h>

#include <math.h>

#include <transform/transform2d.h>
//#include <Transform/DataTypes.h>

namespace NS_Planner
{

  void set_angle(Pose2D* pose, double angle)
  {
	  pose->theta() = angle;
  }

  double getYaw(Pose2D pose)
  {
    //return tf::getYaw(pose.pose.orientation);
//    return NS_Transform::getYaw(pose.pose.orientation);
	  return pose.theta();
  }

  void OrientationFilter::processPath(
      const Pose2D& start,
      std::vector< Pose2D >& path)
  {
    int n = path.size();
    switch(omode_)
    {
      case FORWARD:
        for(int i = 0; i < n - 1; i++)
        {
          pointToNext(path, i);
        }
        break;
      case INTERPOLATE:
        path[0].theta() = start.theta();
        interpolate(path, 0, n - 1);
        break;
      case FORWARDTHENINTERPOLATE:
        for(int i = 0; i < n - 1; i++)
        {
          pointToNext(path, i);
        }

        int i = n - 3;
        double last = getYaw(path[i]);
        while(i > 0)
        {
          double new_angle = getYaw(path[i - 1]);
          double diff = fabs(
              NS_Geometry::NS_Angles::shortest_angular_distance(new_angle,
                                                                last));
          if(diff > 0.35)
            break;
          else
            i--;
        }

        path[0].theta() = start.theta();
        interpolate(path, i, n - 1);
        break;
    }
  }

  void OrientationFilter::pointToNext(
      std::vector< Pose2D >& path, int index)
  {
    double x0 = path[index].x(), y0 = path[index].y(),
        x1 = path[index + 1].x(),
        y1 = path[index + 1].y();

    double angle = sgbot::math::atan2(y1 - y0, x1 - x0);
    set_angle(&path[index], angle);
  }

  void OrientationFilter::interpolate(
      std::vector< Pose2D >& path, int start_index,
      int end_index)
  {
    double start_yaw = getYaw(path[start_index]), end_yaw = getYaw(
        path[end_index]);
    double diff = NS_Geometry::NS_Angles::shortest_angular_distance(start_yaw,
                                                                    end_yaw);
    double increment = diff / (end_index - start_index);
    for(int i = start_index; i <= end_index; i++)
    {
      double angle = start_yaw + increment * i;
      set_angle(&path[i], angle);
    }
  }

}
;
