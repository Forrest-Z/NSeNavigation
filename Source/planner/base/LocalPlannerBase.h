/*
 * LocalPlannerBase.h
 *
 *  Created on: 2016年12月2日
 *      Author: seeing
 */

#ifndef _LOCALPLANNERBASE_H_
#define _LOCALPLANNERBASE_H_

#include <transform/transform2d.h>
#include <DataSet/DataType/Twist.h>

#include "../../costmap/CostmapWrapper.h"

namespace NS_Planner
{

  class LocalPlannerBase
  {
  public:
    LocalPlannerBase()
    {
    }
    ;
    virtual ~LocalPlannerBase()
    {
    }
    ;

  public:
    void initialize(NS_CostMap::CostmapWrapper* costmap_)
    {
      costmap = costmap_;
      onInitialize();
    }
    ;

    virtual void
    onInitialize() = 0;

    virtual bool
    computeVelocityCommands(NS_DataType::Twist& cmd_vel) = 0;

    virtual bool
    isGoalReached() = 0;

    virtual bool
    setPlan(const std::vector< Pose2D >& plan) = 0;

  protected:
    NS_CostMap::CostmapWrapper* costmap;
  };

} /* namespace NS_Planner */

#endif /* NAVIGATION_PLANNER_BASE_LOCALPLANNERBASE_H_ */
