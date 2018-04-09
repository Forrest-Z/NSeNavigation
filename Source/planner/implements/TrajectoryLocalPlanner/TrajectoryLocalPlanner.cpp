#include "TrajectoryLocalPlanner.h"

#include <sys/time.h>
#include <boost/tokenizer.hpp>
#include <cmath>
#include <Console/Console.h>
#include "Algorithm/GoalFunctions.h"
#include <Parameter/Parameter.h>

namespace NS_Planner
{

  TrajectoryLocalPlanner::TrajectoryLocalPlanner()
  {
    world_model_ = NULL;
    tc_ = NULL;
    setup_ = false;
    initialized_ = false;
    odom_helper_ = NULL;
  }

  void TrajectoryLocalPlanner::onInitialize()
  {
    if(!isInitialized())
    {
      NS_NaviCommon::Parameter parameter;

      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
      double sim_time, sim_granularity, angular_sim_granularity;
      int vx_samples, vtheta_samples;
      double pdist_scale, gdist_scale, occdist_scale, heading_lookahead,
          oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
      bool holonomic_robot, dwa, simple_attractor, heading_scoring;
      double heading_scoring_timestep;
      double max_vel_x, min_vel_x;
      double backup_vel;
      double stop_time_buffer;
      std::string world_model_type;
      rotating_to_goal_ = false;

      //initialize the copy of the costmap the controller will use
      costmap_ = costmap->getCostmap();

      /*
       global_frame_ = costmap_ros_->getGlobalFrameID();
       robot_base_frame_ = costmap_ros_->getBaseFrameID();
       */
      parameter.loadConfigurationFile("trajectory_local_planner.xml");

      if(parameter.getParameter("prune_plan", 1) == 1)
      {
        prune_plan_ = true;
      }
      else
      {
        prune_plan_ = false;
      }

      yaw_goal_tolerance_ = parameter.getParameter("yaw_goal_tolerance", 0.1f);
      xy_goal_tolerance_ = parameter.getParameter("xy_goal_tolerance", 0.1f);
      acc_lim_x_ = parameter.getParameter("acc_lim_x", 0.3f);
      acc_lim_y_ = parameter.getParameter("acc_lim_y", 0.3f);
      acc_lim_theta_ = parameter.getParameter("acc_lim_theta", 0.3f);

      stop_time_buffer = parameter.getParameter("stop_time_buffer", 0.2f);

      if(parameter.getParameter("latch_xy_goal_tolerance", 0) == 1)
      {
        latch_xy_goal_tolerance_ = true;
      }
      else
      {
        latch_xy_goal_tolerance_ = false;
      }

      sim_period_ = parameter.getParameter("sim_period", 0.05f);

      sim_time = parameter.getParameter("sim_time", 4.0f);
      sim_granularity = parameter.getParameter("sim_granularity", 0.025f);
      angular_sim_granularity = parameter.getParameter(
          "angular_sim_granularity", 0.025f);
      vx_samples = parameter.getParameter("vx_samples", 20);
      vtheta_samples = parameter.getParameter("vtheta_samples", 40);

      pdist_scale = parameter.getParameter("path_distance_bias", 3.2f);
      gdist_scale = parameter.getParameter("goal_distance_bias", 2.0f);
      occdist_scale = parameter.getParameter("occdist_scale", 0.02f);

      bool meter_scoring;
      if(parameter.getParameter("meter_scoring", 1) == 1)
      {
        meter_scoring = true;
      }
      else
      {
        meter_scoring = false;
      }

      if(meter_scoring)
      {
        //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
        double resolution = costmap_->getResolution();
        logInfo << "trajectoryLocalPlanner resolution = "<< resolution;
        gdist_scale *= resolution;
        pdist_scale *= resolution;
        occdist_scale *= resolution;
      }
      else
      {
        printf(
            "Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settins robust against changes of costmap resolution.\n");
      }

      heading_lookahead = parameter.getParameter("heading_lookahead", 0.325f);
      oscillation_reset_dist = parameter.getParameter("oscillation_reset_dist",
                                                      0.05f);
      escape_reset_dist = parameter.getParameter("escape_reset_dist", 0.05f);
      escape_reset_theta = parameter.getParameter("escape_reset_theta",
                                                  (float)M_PI_4);
      if(parameter.getParameter("holonomic_robot", 0) == 1)
      {
        holonomic_robot = true;
      }
      else
      {
        holonomic_robot = false;
      }
      max_vel_x = parameter.getParameter("max_vel_x", 0.2f);
      min_vel_x = parameter.getParameter("min_vel_x", 0.1f);

      double max_rotational_vel;
      max_rotational_vel = parameter.getParameter("max_rotational_vel", 0.3f);
      max_vel_th_ = max_rotational_vel;
//      min_vel_th_ = -1.0 * max_rotational_vel;
      min_vel_th_ = parameter.getParameter("min_rotational_vel", 0.1f);
      min_in_place_vel_th_ = parameter.getParameter(
          "min_in_place_rotational_vel", 0.1f);
      reached_goal_ = false;
      backup_vel = -0.1;
      backup_vel = parameter.getParameter("escape_vel", -0.1f);

      if(backup_vel >= 0.0)
        printf(
            "You've specified a positive escape velocity. "
            "This is probably not what you want and will cause the robot to move forward instead of backward. "
            "You should probably change your escape_vel parameter to be negative\n");

      if(parameter.getParameter("dwa", 1) == 1)
      {
        dwa = true;
      }
      else
      {
        dwa = false;
      }
      if(parameter.getParameter("heading_scoring", 1) == 1)
      {
        heading_scoring = true;
      }
      else
      {
        heading_scoring = false;
      }
      heading_scoring_timestep = parameter.getParameter(
          "heading_scoring_timestep", 0.8f);

      request_times = parameter.getParameter("request_times", 3);

      int simple_attractor_i = parameter.getParameter("simple_attractor", 0);
      simple_attractor = bool(simple_attractor_i);

      world_model_ = new CostmapModel(*costmap_);

      footprint_spec_ = costmap->getRobotFootprint();
      if(footprint_spec_.size() == 0){
    	  logWarn << "footprint_spec_.size = 0 ";
      }
      logInfo << "footprint_spec_ size = "<<footprint_spec_.size();
      double circums_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
      double inscribe_radius = costmap->getLayeredCostmap()->getInscribedRadius();
      odom_helper_ = new OdometryHelper();

      std::vector< double > y_vels = std::vector< double >(0);
      y_vels.clear();

      tc_ = new TrajectoryPlanner(*world_model_, *costmap_, footprint_spec_,
                                  acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                                  sim_time, sim_granularity, vx_samples,
                                  vtheta_samples, pdist_scale, gdist_scale,
                                  occdist_scale, heading_lookahead,
                                  oscillation_reset_dist, escape_reset_dist,
                                  escape_reset_theta, holonomic_robot,
                                  max_vel_x, min_vel_x, max_vel_th_,
                                  min_vel_th_, min_in_place_vel_th_, backup_vel,
                                  dwa, heading_scoring,
                                  heading_scoring_timestep, meter_scoring,
                                  simple_attractor, y_vels, stop_time_buffer,
                                  sim_period_, angular_sim_granularity,
                                  circums_radius, inscribe_radius);

      initialized_ = true;

    }
    else
    {
      printf("This planner has already been initialized, doing nothing\n");
    }
  }

  TrajectoryLocalPlanner::~TrajectoryLocalPlanner()
  {
    //make sure to clean things up
    if(tc_ != NULL)
      delete tc_;

    if(world_model_ != NULL)
      delete world_model_;

    if(odom_helper_ != NULL)
      delete odom_helper_;
  }

  bool TrajectoryLocalPlanner::stopWithAccLimits(
      const Pose2D& global_pose,
      const Velocity2D& robot_vel,
      Velocity2D& cmd_vel)
  {
    //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
    //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
    double vx = sign(robot_vel.linear) * std::max(
        0.0, (fabs(robot_vel.linear) - acc_lim_x_ * sim_period_));
//    double vy = sign(robot_vel.getOrigin().y()) * std::max(
//        0.0, (fabs(robot_vel.getOrigin().y()) - acc_lim_y_ * sim_period_));
    double vy = 0.0;
    double vel_yaw = robot_vel.angular;
    double vth = sign(vel_yaw) * std::max(
        0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

    //we do want to check whether or not the command is valid
    double yaw = global_pose.theta();
    bool valid_cmd = tc_->checkTrajectory(global_pose.x(),
                                          global_pose.y(), yaw,
                                          robot_vel.linear,
                                          0.0, vel_yaw,
                                          vx, vy, vth);

    //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
    if(valid_cmd)
    {
      printf("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f\n", vx, vy,
             vth);
      cmd_vel.linear = vx;
//      cmd_vel.linear.y = vy;
      cmd_vel.angular = vth;
      return true;
    }else {
    	logInfo << "stop with acc limit failed";
    }

    cmd_vel.linear = 0.0;
//    cmd_vel.linear.y = 0.0;
    cmd_vel.angular = 0.0;
    return false;
  }

  bool TrajectoryLocalPlanner::rotateToGoal(
      const Pose2D& global_pose,
      const Velocity2D& robot_vel,
      double goal_th, Velocity2D& cmd_vel)
  {
    double yaw = global_pose.theta();
    double vel_yaw = robot_vel.angular;
    cmd_vel.linear = 0;

    double ang_diff = angleDiff(goal_th - yaw);

    double v_theta_samp =
        ang_diff > 0.0 ? std::min(max_vel_th_,
                                  std::max(min_in_place_vel_th_, ang_diff)) : std::max(
            min_vel_th_, std::min(-1.0 * min_in_place_vel_th_, ang_diff));

    //take the acceleration limits of the robot into account
    double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
    double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

    v_theta_samp = sign(v_theta_samp) * std::min(
        std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff));

    v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop,
                                                 fabs(v_theta_samp));

    // Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
    v_theta_samp =
        v_theta_samp > 0.0 ? std::min(
            max_vel_th_, std::max(min_in_place_vel_th_, v_theta_samp)) : std::max(
            min_vel_th_, std::min(-1.0 * min_in_place_vel_th_, v_theta_samp));

    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = tc_->checkTrajectory(global_pose.x(),
                                          global_pose.y(), yaw,
                                          robot_vel.linear,
                                          0.0, vel_yaw,
                                          0.0, 0.0, v_theta_samp);

    printf("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d\n",
           v_theta_samp, valid_cmd);

    if(valid_cmd)
    {
      cmd_vel.angular = v_theta_samp;
      return true;
    }else {
    	logInfo << "rotate to goal failed ";
    }

    cmd_vel.angular = 0.0;
    return false;

  }

  bool TrajectoryLocalPlanner::setPlan(
      const std::vector< Pose2D >& orig_global_plan)
  {
    if(!isInitialized())
    {
      printf(
          "This planner has not been initialized, please call initialize() before using this planner\n");
      return false;
    }
    printf("trajectory::setplan---------------------\n");
    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;

    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;
    //reset the at goal flag
    reached_goal_ = false;
    return true;
  }

  bool TrajectoryLocalPlanner::computeVelocityCommands(
		  Velocity2D& cmd_vel)
  {
    if(!isInitialized())
    {
      printf(
          "This planner has not been initialized, please call initialize() before using this planner\n");
      return false;
    }

    std::vector < Pose2D > local_plan;
    Pose2D global_pose;

    int times = 0;
    bool getRobotPoseReady = 0;
    while(times != request_times)
    {
      if(costmap->getRobotPose(global_pose) == true)
      {
        printf("trajectory compute vel get robot pose success\n");
        getRobotPoseReady = 1;
        break;
      }
      ++times;
    }

    if(!getRobotPoseReady)
    {
      printf("trajectory compute vel get robot pose failure!!!!!!!!!!!!!!!!\n");
      return false;
    }

    std::vector < Pose2D > transformed_plan;

    logInfo<<"remove transformed plan,maybe prune_plan can be removed at the meantime";

    if(prune_plan_)
      prunePlan(global_pose, global_plan_, global_plan_);

    printf("compute vel global_pose x = %.4f,y = %.4f,w = %.4f\n",
           global_pose.x(), global_pose.y(),
           global_pose.theta());


    Velocity2D drive_cmds;

    Velocity2D robot_vel;
    odom_helper_->getRobotVel(robot_vel);

    printf(
        "odom_helper.get robot_vel x = %.4f, y = %.4f , yaw = %.4f , global_plan size = %d\n",
        robot_vel.linear, 0.0,
        robot_vel.angular, global_plan_.size());


    if(global_plan_.empty())
    {
      printf("global plan is empty return false!");
      return false;
    }
//    NS_Transform::Stamped < NS_Transform::Pose > goal_point;
//    NS_Transform::poseStampedMsgToTF(transformed_plan.back(), goal_point);

    Pose2D goal_point = global_plan_.back();


    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.x();
    double goal_y = goal_point.y();
    printf("compute vel goal_x = %.4f,goal_y = %.4f\n", goal_x, goal_y);
    double yaw = goal_point.theta();

//    FILE* global_pose_file;
//    global_pose_file = fopen("/tmp/global_pose.log", "a+");
//    fprintf(global_pose_file, "%.4f %.4f %.4f\n", global_pose.getOrigin().x(),
//            global_pose.getOrigin().y(),
//            NS_Transform::getYaw(global_pose.getRotation()));
//    fclose(global_pose_file);

    double goal_th = yaw;

    //check to see if we've reached the goal position
    if(xy_tolerance_latch_ || (getGoalPositionDistance(global_pose, goal_x,
                                                       goal_y) <= xy_goal_tolerance_))
    {
      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
      //just rotate in place
      if(latch_xy_goal_tolerance_)
      {
        xy_tolerance_latch_ = true;
      }
      double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
      //check to see if the goal orientation has been reached
      if(fabs(angle) <= yaw_goal_tolerance_)
      {
        //set the velocity command to zero
        cmd_vel.linear = 0.0;
        cmd_vel.angular = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
        reached_goal_ = true;
      }
      else
      {
        //we need to call the next two lines to make sure that the trajectory
        //planner updates its path distance and goal distance grids

        tc_->updatePlan(global_plan_);
        Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);

        //copy over the odometry information
        Odometry base_odom;
        odom_helper_->getOdom(base_odom);

        //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
        if(!rotating_to_goal_ && !NS_Planner::stopped(base_odom,
                                                      rot_stopped_velocity_,
                                                      trans_stopped_velocity_))
        {
          if(!stopWithAccLimits(global_pose, robot_vel, cmd_vel))
          {
            return false;
          }
        }
        //if we're stopped... then we want to rotate to goal
        else
        {
          //set this so that we know its OK to be moving
          rotating_to_goal_ = true;
          if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel))
          {
            return false;
          }
        }
      }

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }

    tc_->updatePlan(global_plan_);

    //compute what trajectory to drive along
    Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);


    //pass along drive commands
    cmd_vel.linear = drive_cmds.linear;
//    cmd_vel.linear.y = 0.0;
    cmd_vel.angular = drive_cmds.angular;

    //if we cannot move... tell someone
    if(path.cost_ < 0)
    {
      printf(
          "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.\n");
      local_plan.clear();
      return false;
    }

    printf(
        "A valid velocity command of (%.2f, 0.0f, %.2f) was found for this cycle.\n",
        cmd_vel.linear, cmd_vel.angular);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i)
    {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);
//      NS_Transform::Stamped < NS_Transform::Pose > p = NS_Transform::Stamped < NS_Transform::Pose > (NS_Transform::Pose(
//          NS_Transform::createQuaternionFromYaw(p_th),
//          NS_Transform::Point(p_x, p_y, 0.0)), NS_NaviCommon::Time::now(), "");
      Pose2D pose;
      pose.x() = p_x;
      pose.y() = p_y;
      pose.theta() = p_th;
      local_plan.push_back(pose);
    }
    ///TODO need to be visualized
    logInfo << "publish local plan from trajectory";
    return true;
  }

  bool TrajectoryLocalPlanner::checkTrajectory(double vx_samp, double vy_samp,
                                               double vtheta_samp,
                                               bool update_map)
  {
	Pose2D global_pose;
    if(costmap->getRobotPose(global_pose))
    {
      if(update_map)
      {
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector < Pose2D > plan;
//        Pose2D pose_msg;
//        NS_Transform::poseStampedTFToMsg(global_pose, pose_msg);
        plan.push_back(global_pose);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
      Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->checkTrajectory(
          global_pose.x(), global_pose.y(),
          global_pose.theta(),
          base_odom.velocity2d.linear, 0.0,
          base_odom.velocity2d.angular, vx_samp, vy_samp, vtheta_samp);

    }
    printf(
        "Failed to get the pose of the robot. No trajectories will pass as legal in this case.\n");
    return false;
  }

  double TrajectoryLocalPlanner::scoreTrajectory(double vx_samp, double vy_samp,
                                                 double vtheta_samp,
                                                 bool update_map)
  {
    // Copy of checkTrajectory that returns a score instead of True / False
	  Pose2D global_pose;
    if(costmap->getRobotPose(global_pose))
    {
      if(update_map)
      {
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector < Pose2D > plan;
//        Pose2D pose_msg;
//        NS_Transform::poseStampedTFToMsg(global_pose, pose_msg);
        plan.push_back(global_pose);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
      Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->scoreTrajectory(
          global_pose.x(), global_pose.y(),
          global_pose.theta(),
          base_odom.velocity2d.linear, 0.0,
          base_odom.velocity2d.angular, vx_samp, vy_samp, vtheta_samp);

    }
    printf(
        "Failed to get the pose of the robot. No trajectories will pass as legal in this case.\n");
    return -1.0;
  }

  bool TrajectoryLocalPlanner::isGoalReached()
  {
    if(!isInitialized())
    {
      printf(
          "This planner has not been initialized, please call initialize() before using this planner.\n");
      return false;
    }
    printf("reached_goal??????????????????????????? = %d\n", reached_goal_);
    //return flag set in controller
    return reached_goal_;
  }
}
;
