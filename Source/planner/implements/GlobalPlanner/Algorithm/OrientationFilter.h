#ifndef _ORIENTATION_FILTER_H_
#define _ORIENTATION_FILTER_H_

#include <transform/transform2d.h>

namespace NS_Planner
{

  enum OrientationMode
  {
    NONE,
    FORWARD,
    INTERPOLATE,
    FORWARDTHENINTERPOLATE
  };

  class OrientationFilter
  {
  public:
    OrientationFilter()
        : omode_(NONE)
    {
    }

    virtual void
    processPath(const sgbot::tf::Pose2D& start,
                std::vector< sgbot::tf::Pose2D >& path);

    void
    pointToNext(std::vector< sgbot::tf::Pose2D >& path, int index);
    void
    interpolate(std::vector< sgbot::tf::Pose2D >& path, int start_index,
                int end_index);

    void setMode(OrientationMode new_mode)
    {
      omode_ = new_mode;
    }
    void setMode(int new_mode)
    {
      setMode((OrientationMode)new_mode);
    }
  protected:
    OrientationMode omode_;
  };

} //end namespace global_planner
#endif
