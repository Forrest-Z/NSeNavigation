#ifndef _BASE_LOCAL_PLANNER_GOAL_FUNCTIONS_H_
#define _BASE_LOCAL_PLANNER_GOAL_FUNCTIONS_H_

#include <DataSet/DataType/Odometry.h>
#include <DataSet/DataType/Path.h>
#include <transform/transform2d.h>
#include <DataSet/DataType/Twist.h>
#include <DataSet/DataType/Point.h>

#include <string>
#include <cmath>

#include <Geometry/Angles.h>
#include <Transform/DataTypes.h>
#include "../../../../costmap/costmap_2d/CostMap2D.h"

namespace NS_Planner
{

  /**
   * @brief  return squared distance to check if the goal position has been achieved
   * @param  global_pose The pose of the robot in the global frame
   * @param  goal_x The desired x value for the goal
   * @param  goal_y The desired y value for the goal
   * @return distance to goal
   */
  double
  getGoalPositionDistance(
      const Pose2D& global_pose,
      double goal_x, double goal_y);

  /**
   * @brief  return angle difference to goal to check if the goal orientation has been achieved
   * @param  global_pose The pose of the robot in the global frame
   * @param  goal_x The desired x value for the goal
   * @param  goal_y The desired y value for the goal
   * @return angular difference
   */
  double
  getGoalOrientationAngleDifference(
      const Pose2D& global_pose,
      double goal_th);

  /**
   * @brief  Trim off parts of the global plan that are far enough behind the robot
   * @param global_pose The pose of the robot in the global frame
   * @param plan The plan to be pruned
   * @param global_plan The plan to be pruned in the frame of the planner
   */
  void
  prunePlan(const Pose2D& global_pose,
            std::vector< Pose2D >& plan,
            std::vector< Pose2D >& global_plan);


  /**
   * @brief  Returns last pose in plan
   * @param tf A reference to a transform listener
   * @param global_plan The plan being followed
   * @param global_frame The global frame of the local planner
   * @param goal_pose the pose to copy into
   * @return True if achieved, false otherwise
   */
  bool
  getGoalPose(const std::vector< Pose2D >& global_plan,
              Pose2D &goal_pose);

  /**
   * @brief  Check if the goal pose has been achieved
   * @param tf A reference to a transform listener
   * @param global_plan The plan being followed
   * @param costmap_ros A reference to the costmap object being used by the planner
   * @param global_frame The global frame of the local planner
   * @param base_odom The current odometry information for the robot
   * @param rot_stopped_vel The rotational velocity below which the robot is considered stopped
   * @param trans_stopped_vel The translational velocity below which the robot is considered stopped
   * @param xy_goal_tolerance The translational tolerance on reaching the goal
   * @param yaw_goal_tolerance The rotational tolerance on reaching the goal
   * @return True if achieved, false otherwise
   */
  bool
  isGoalReached(const std::vector< Pose2D >& global_plan,
                const NS_CostMap::Costmap2D& costmap,
                Pose2D& global_pose,
                const NS_DataType::Odometry& base_odom, double rot_stopped_vel,
                double trans_stopped_vel, double xy_goal_tolerance,
                double yaw_goal_tolerance);

  /**
   * @brief  Check whether the robot is stopped or not
   * @param base_odom The current odometry information for the robot
   * @param rot_stopped_velocity The rotational velocity below which the robot is considered stopped
   * @param trans_stopped_velocity The translational velocity below which the robot is considered stopped
   * @return True if the robot is stopped, false otherwise
   */
  bool
  stopped(const NS_DataType::Odometry& base_odom,
          const double& rot_stopped_velocity,
          const double& trans_stopped_velocity);
}
;
#endif
