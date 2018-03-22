#ifndef _ORIENTATION_FILTER_H_
#define _ORIENTATION_FILTER_H_

#include <transform/transform2d.h>

using namespace sgbot;
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
    processPath(const Pose2D& start,
                std::vector< Pose2D >& path);

    void
    pointToNext(std::vector< Pose2D >& path, int index);
    void
    interpolate(std::vector< Pose2D >& path, int start_index,
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
