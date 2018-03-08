#include <Console/Console.h>
#include "GoalFunctions.h"
#include <math.h>

#include <Service/ServiceType/ServiceTransform.h>
#include <Service/Client.h>

namespace NS_Planner
{

  double getGoalPositionDistance(
      const sgbot::tf::Pose2D& global_pose,
      double goal_x, double goal_y)
  {
    return hypot(goal_x - global_pose.x,
                 goal_y - global_pose.y);
  }

  double getGoalOrientationAngleDifference(
      const sgbot::tf::Pose2D& global_pose,
      double goal_th)
  {
//    double yaw = NS_Transform::getYaw(global_pose);
	  double yaw = global_pose.theta;
    return NS_Geometry::NS_Angles::shortest_angular_distance(yaw, goal_th);
  }

  void prunePlan(const sgbot::tf::Pose2D& global_pose,
                 std::vector< sgbot::tf::Pose2D >& plan,
                 std::vector< sgbot::tf::Pose2D >& global_plan)
  {
    assert(global_plan.size() >= plan.size());
    std::vector< sgbot::tf::Pose2D >::iterator it = plan.begin();
    std::vector< sgbot::tf::Pose2D >::iterator global_it = global_plan.begin();
    while(it != plan.end())
    {
      const sgbot::tf::Pose2D& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose.x - w.x;
      double y_diff = global_pose.y - w.y;
      double distance_sq = x_diff * x_diff + y_diff * y_diff;
      if(distance_sq < 1)
      {
        printf("Nearest waypoint to <%f, %f> is <%f, %f>\n",
               global_pose.x, global_pose.y,
               w.x, w.y);
        break;
      }
      it = plan.erase(it);
      global_it = global_plan.erase(global_it);
    }
  }


  bool getGoalPose(const std::vector< sgbot::tf::Pose2D >& global_plan,
                   sgbot::tf::Pose2D& goal_pose)
  {
    if(global_plan.empty())
    {
      printf("Received plan with zero length\n");
      return false;
    }

    goal_pose = global_plan.back();
    printf("get goal pose global_plan size = %d\n", global_plan.size());

//    for(int i = 0; i < global_plan.size(); ++i)
//    {
//      printf("global_plan[%d],x = %.4f,y=%.4f\n", i,
//             global_plan[i].pose.position.x, global_plan[i].pose.position.y);
//    }
//    poseStampedMsgToTF(plan_goal_pose, goal_pose);

//    if(0)
//    {
//      //NS_Transform::StampedTransform transform;
//      /*
//       tf.waitForTransform(global_frame, ros::Time::now(),
//       plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
//       plan_goal_pose.header.frame_id, ros::Duration(0.5));
//       tf.lookupTransform(global_frame, ros::Time(),
//       plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
//       plan_goal_pose.header.frame_id, transform);
//       */
//      NS_Service::Client < NS_ServiceType::ServiceTransform > map_tf_cli(
//          "ODOM_MAP_TF");
//      NS_ServiceType::ServiceTransform map_transform;
//      if(map_tf_cli.call(map_transform) == false)
//      {
//        printf("Get map transform failure!\n");
//        return false;
//      }
//
//      NS_Transform::Transform transform;
//
//      //poseStampedMsgToTF(plan_goal_pose, goal_pose);
//      NS_Transform::transformMsgToTF(map_transform.transform, transform);
//
//      goal_pose.setData(transform * goal_pose);
//    }

    return true;
  }

  bool isGoalReached(
      const std::vector< sgbot::tf::Pose2D >& global_plan,
      const NS_CostMap::Costmap2D& costmap __attribute__((unused)),
      sgbot::tf::Pose2D& global_pose,
      const NS_DataType::Odometry& base_odom, double rot_stopped_vel,
      double trans_stopped_vel, double xy_goal_tolerance,
      double yaw_goal_tolerance)
  {
    //we assume the global goal is the last point in the global plan
	sgbot::tf::Pose2D goal_pose;
    getGoalPose(global_plan, goal_pose);

    double goal_x = goal_pose.x;
    double goal_y = goal_pose.y;
    double goal_th = goal_pose.theta;

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

  bool stopped(const NS_DataType::Odometry& base_odom,
               const double& rot_stopped_velocity,
               const double& trans_stopped_velocity)
  {
    return fabs(base_odom.twist.angular.z) <= rot_stopped_velocity && fabs(
        base_odom.twist.linear.x) <= trans_stopped_velocity && fabs(
        base_odom.twist.linear.y) <= trans_stopped_velocity;
  }
}
;
