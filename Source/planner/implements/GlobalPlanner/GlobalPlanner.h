#ifndef _GLOBALPLANNER_H_
#define _GLOBALPLANNER_H_

#include "../../base/GlobalPlannerBase.h"

#include "Algorithm/PotentialCalculator.h"
#include "Algorithm/Expander.h"
#include "Algorithm/Traceback.h"
#include "Algorithm/OrientationFilter.h"

namespace NS_Planner
{

  class Expander;
  class GridPath;

  class GlobalPlanner: public GlobalPlannerBase
  {
  public:
    GlobalPlanner();
    virtual
    ~GlobalPlanner();

    void
    onInitialize();

    bool
    makePlan(const sgbot::tf::Pose2D& start,
             const sgbot::tf::Pose2D& goal,
             std::vector< sgbot::tf::Pose2D >& plan);

    bool
    getPlanFromPotential(double start_x, double start_y, double end_x,
                         double end_y, const sgbot::tf::Pose2D& goal,
                         std::vector< sgbot::tf::Pose2D >& plan);


  protected:
    bool initialized_, allow_unknown_; //, visualize_potential_;

  private:
    void
    mapToWorld(double mx, double my, double& wx, double& wy);
    bool
    worldToMap(double wx, double wy, double& mx, double& my);
    void
    clearRobotCell(unsigned int mx, unsigned int my);

    double planner_window_x_, planner_window_y_, default_tolerance_;

    boost::mutex mutex_;

    PotentialCalculator* p_calc_;
    Expander* planner_;
    Traceback* path_maker_;
    OrientationFilter* orientation_filter_;
    bool publish_potential_;
    int publish_scale_;

    void
    outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
    unsigned char* cost_array_;
    float* potential_array_;
    unsigned int start_x_, start_y_, end_x_, end_y_;

    float convert_offset_;
  };

} /* namespace NS_Planner */

#endif /* NAVIGATION_PLANNER_IMPLEMENTS_GLOBALPLANNER_GLOBALPLANNER_H_ */
