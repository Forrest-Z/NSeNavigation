#include <Console/Console.h>
#include "GoalFunctions.h"
#include <math.h>

#include <Service/Client.h>

namespace NS_Planner
{

  double getGoalPositionDistance(
      const Pose2D& global_pose,
      double goal_x, double goal_y)
  {
    return hypot(goal_x - global_pose.x(),
                 goal_y - global_pose.y());
  }

  double getGoalOrientationAngleDifference(
      const Pose2D& global_pose,
      double goal_th)
  {
//    double yaw = NS_Transform::getYaw(global_pose);
	  double yaw = global_pose.theta();
    return angleDiff(goal_th - yaw);
  }

  void prunePlan(const Pose2D& global_pose,
                 std::vector< Pose2D >& plan,
                 std::vector< Pose2D >& global_plan)
  {
    assert(global_plan.size() >= plan.size());
    std::vector< Pose2D >::iterator it = plan.begin();
    std::vector< Pose2D >::iterator global_it = global_plan.begin();
    while(it != plan.end())
    {
      const Pose2D& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose.x() - w.x();
      double y_diff = global_pose.y() - w.y();
      double distance_sq = x_diff * x_diff + y_diff * y_diff;
      if(distance_sq < 1)
      {
        printf("Nearest waypoint to <%f, %f> is <%f, %f>\n",
               global_pose.x(), global_pose.y(),
               w.x(), w.y());
        break;
      }
      it = plan.erase(it);
      global_it = global_plan.erase(global_it);
    }
  }


  bool getGoalPose(const std::vector< Pose2D >& global_plan,
                   Pose2D& goal_pose)
  {
    if(global_plan.empty())
    {
      printf("Received plan with zero length\n");
      return false;
    }

    goal_pose = global_plan.back();
    printf("get goal pose global_plan size = %d\n", global_plan.size());

    return true;
  }

  bool isGoalReached(
      const std::vector< Pose2D >& global_plan,
      const NS_CostMap::Costmap2D& costmap __attribute__((unused)),
      Pose2D& global_pose,
      const Odometry& base_odom, double rot_stopped_vel,
      double trans_stopped_vel, double xy_goal_tolerance,
      double yaw_goal_tolerance)
  {
    //we assume the global goal is the last point in the global plan
	Pose2D goal_pose;
    getGoalPose(global_plan, goal_pose);

    double goal_x = goal_pose.x();
    double goal_y = goal_pose.y();
    double goal_th = goal_pose.theta();

    //check to see if we've reached the goal position
    if(getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance)
    {
      //check to see if the goal orientation has been reached
      if(fabs(getGoalOrientationAngleDifference(global_pose, goal_th)) <= yaw_goal_tolerance)
      {
        //make sure that we're actually stopped before returning success
        if(stopped(base_odom, rot_stopped_vel, trans_stopped_vel))
          return true;
      }
    }

    return false;
  }

  bool stopped(const Odometry& base_odom,
               const double& rot_stopped_velocity,
               const double& trans_stopped_velocity)
  {
    return fabs(base_odom.velocity2d.angular) <= rot_stopped_velocity && fabs(
        base_odom.velocity2d.linear) <= trans_stopped_velocity;
//		&& fabs(base_odom.twist.linear.y) <= trans_stopped_velocity;
  }
}
;
