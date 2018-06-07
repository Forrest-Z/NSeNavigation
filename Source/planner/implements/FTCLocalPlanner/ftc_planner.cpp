

#include "ftc_planner.h"


namespace NS_Planner
{

    FTCPlanner::FTCPlanner()
    {
    }

    void FTCPlanner::onInitialize()
    {

        first_setPlan_ = true;
        rotate_to_global_plan_ = false;
        goal_reached_ = false;
        stand_at_goal_ = false;
        cmd_vel_angular_z_rotate_ = 0.f;
        cmd_vel_linear_x_ = 0.f;
        cmd_vel_angular_z_ = 0.f;

        NS_NaviCommon::Parameter parameter;
        parameter.loadConfigurationFile("ftc_planner.xml");
        position_accuracy = parameter.getParameter("position_accuracy", 0.1f);
        rotation_accuracy = parameter.getParameter("rotation_accuracy", 0.1f);
        max_rotation_vel = parameter.getParameter("max_rotation_vel",0.8f);
        min_rotation_vel = parameter.getParameter("min_rotation_vel",0.2f);
        max_x_vel = parameter.getParameter("max_x_vel", 0.2f);
        sim_time = parameter.getParameter("sim_time",0.4f);

        acceleration_x = parameter.getParameter("acceleration_x",0.1f);
        acceleration_z = parameter.getParameter("acceleration_z",0.1f);
        slow_down_factor = parameter.getParameter("slow_down_factor",1.f);
        local_planner_frequence = parameter.getParameter("local_planner_frequence",20.f);
        logInfo <<"ftc planner initialized";
    }

    bool FTCPlanner::getPoseInPlan(const std::vector<sgbot::Pose2D>& global_plan,sgbot::Pose2D& goal_pose,int plan_point){
    	if(global_plan.empty()){
    		logError<<"global plan is empty get nothing";
    		return false;
    	}
    	if(plan_point >= global_plan.size()){
    		logError<<"get plan point"<<plan_point<<" >= global plan size"<<global_plan.size();
    		return false;
    	}
    	goal_pose = global_plan.at(plan_point);
    	return true;
    }

    bool FTCPlanner::setPlan(const std::vector<sgbot::Pose2D>& plan)
    {
        global_plan_ = plan;

        //First start of the local plan. First global plan.
        bool first_use = false;
        if(first_setPlan_)
        {

            first_setPlan_ = false;
            getPoseInPlan(global_plan_,old_goal_pose_,global_plan_.size()-1);
            first_use = true;
        }

        getPoseInPlan(global_plan_,goal_pose_,global_plan_.size()-1);
        //Have the new global plan an new goal, reset. Else dont reset.
        if(sgbot::distance(old_goal_pose_,goal_pose_) < position_accuracy && !first_use
        		&& angleDiff(old_goal_pose_.theta(),goal_pose_.theta()) < rotation_accuracy){
        	logInfo << "old goal == goal";
        }
        else
        {
            //Rotate to first global plan point.
            rotate_to_global_plan_ = true;
            goal_reached_ = false;
            stand_at_goal_ = false;
            logInfo << "FTCPlanner: New Goal. Start new routine.";
        }
        logInfo << "set plan size = "<<global_plan_.size()<<"get goal pose = "<<goal_pose_.x()<<" , "<<goal_pose_.y()<<" , "<<goal_pose_.theta();
        old_goal_pose_ = goal_pose_;

        return true;
    }

    bool FTCPlanner::computeVelocityCommands(sgbot::Velocity2D& cmd_vel)
    {

        sgbot::Pose2D current_pose;
        costmap->getRobotPose(current_pose);
        logInfo <<"ftc planner get pose = "<<current_pose.x()<<" , "<<current_pose.y()<<" , "<<current_pose.theta();
        int max_point = 0;
        //First part of the routine. Rotatio to the first global plan orientation.
        if(rotate_to_global_plan_)
        {
        	logInfo <<"first part rotate to gloal plan orientation";
            float angle_to_global_plan = calculateGlobalPlanAngle(current_pose, global_plan_, checkMaxDistance(current_pose));
            rotate_to_global_plan_ = rotateToOrientation(angle_to_global_plan, cmd_vel, rotation_accuracy);
        }
        //Second part of the routine. Drive alonge the global plan.
        else
        {
        	float distance = sgbot::distance(goal_pose_,current_pose);
        	logInfo << "second part is near enough distance = "<<distance;
            //Check if robot near enough to global goal.
            if(distance > position_accuracy && !stand_at_goal_)
            {

                if(fabs(calculateGlobalPlanAngle(current_pose, global_plan_, checkMaxDistance(current_pose)) > 1.2))
                {
                    logInfo << ("FTCPlanner: Excessive deviation from global plan orientation. Start routine new.");
                    rotate_to_global_plan_ = true;
                }

                max_point = driveToward(current_pose, cmd_vel);

                if(!checkCollision(max_point))
                {
                	logInfo <<"collision true";
                    return false;
                }
            }
            //Third part of the routine. Rotate at goal to goal orientation.
            else
            {
            	logInfo << "third part rotate to the goal";
                if(!stand_at_goal_)
                {
                    logInfo << ("FTCPlanner: Stand at goal. Rotate to goal orientation.");
                }
                stand_at_goal_ = true;


                //Get the goal orientation.
                float angle_to_global_plan = angleDiff(current_pose.theta(),goal_pose_.theta());
                //Rotate until goalorientation is reached.
                if(!rotateToOrientation(angle_to_global_plan, cmd_vel, rotation_accuracy))
                {
                	logInfo <<"goal reached";
                    goal_reached_ = true;
                }
                cmd_vel.linear = 0;
            }
        }

        publishPlan(max_point);
        return true;
    }

    int FTCPlanner::checkMaxDistance(sgbot::Pose2D current_pose)
    {
        int max_point = 0;
        sgbot::Pose2D x_pose;

        for (unsigned int i = 0; i < global_plan_.size(); i++)
        {
            getPoseInPlan(global_plan_,x_pose,i);
            float distance = sgbot::distance(x_pose,current_pose);

            max_point = i-1;
            //If distance higher than maximal moveable distance in sim_time.
            if(distance > (max_x_vel*sim_time))
            {
                break;
            }
        }
        if(max_point < 0)
        {
            max_point = 0;
        }
        logInfo <<"max point = "<<max_point;
        return max_point;
    }

    int FTCPlanner::checkMaxAngle(int points, sgbot::Pose2D current_pose)
    {
        int max_point = points;
        double angle = 0;
        for(int i = max_point; i >= 0; i--)
        {
            angle = calculateGlobalPlanAngle(current_pose, global_plan_, i);

            max_point = i;
            //check if the angle is moveable
            if(fabs(angle) < max_rotation_vel*sim_time)
            {
                break;
            }
        }
        return max_point;
    }

    float FTCPlanner::calculateGlobalPlanAngle(sgbot::Pose2D current_pose, const std::vector<sgbot::Pose2D>& plan, int point)
    {
        if(point >= (int)plan.size())
        {
            point = plan.size()-1;
        }
        float angle = 0.f;
        float current_th = current_pose.theta();
        for(int i = 0; i <= point; i++)
        {
            sgbot::Pose2D x_pose;
            x_pose=global_plan_.at(point);

            //Calculate the angles between robotpose and global plan point pose
            float angle_to_goal = sgbot::math::atan2(x_pose.y() - current_pose.y(),
                                         x_pose.x() - current_pose.x());
            angle += angle_to_goal;
        }

        //average
        angle = angle/(point+1);
        logInfo << "angle average = "<<angle;
        float angle_diff = angleDiff(current_th, angle);
        logInfo <<"global plan angle diff = "<<angle_diff;
        return angle_diff;
    }

    bool FTCPlanner::rotateToOrientation(float angle, sgbot::Velocity2D& cmd_vel, float accuracy)
    {

        if((cmd_vel_linear_x_  - 0.1)  >= 0){
            cmd_vel.linear = cmd_vel_linear_x_ - 0.1;
            cmd_vel_linear_x_ = cmd_vel_linear_x_ - 0.1;
        }
        if(fabs(angle) > accuracy)
        {
            //Slow down
            if(max_rotation_vel >= fabs(angle) * (acceleration_z+slow_down_factor))
            {
                logInfo << "FTCPlanner: Slow down.";
                if(angle < 0)
                {
                    if(cmd_vel_angular_z_rotate_ >= -min_rotation_vel)
                    {
                        cmd_vel_angular_z_rotate_ = - min_rotation_vel;
                        cmd_vel.angular = cmd_vel_angular_z_rotate_;

                    }
                    else
                    {
                        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ + acceleration_z/local_planner_frequence;
                        cmd_vel.angular = cmd_vel_angular_z_rotate_;
                    }
                }
                if(angle > 0)
                {
                    if(cmd_vel_angular_z_rotate_  <= min_rotation_vel)
                    {
                        cmd_vel_angular_z_rotate_ =  min_rotation_vel;
                        cmd_vel.angular = cmd_vel_angular_z_rotate_;

                    }
                    else
                    {
                        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ - acceleration_z/local_planner_frequence;
                        cmd_vel.angular = cmd_vel_angular_z_rotate_;
                    }
                }
            }
            else
            {
                //Speed up
                if(fabs(cmd_vel_angular_z_rotate_) < max_rotation_vel)
                {
                    logInfo << ("FTCPlanner: Speeding up");
                    if(angle < 0)
                    {
                        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ - acceleration_z/local_planner_frequence;

                        if(fabs(cmd_vel_angular_z_rotate_) > max_rotation_vel)
                        {
                            cmd_vel_angular_z_rotate_ = - max_rotation_vel;
                        }
                        cmd_vel.angular = cmd_vel_angular_z_rotate_;
                    }
                    if(angle > 0)
                    {
                        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ + acceleration_z/local_planner_frequence;

                        if(fabs(cmd_vel_angular_z_rotate_) > max_rotation_vel)
                        {
                            cmd_vel_angular_z_rotate_ = max_rotation_vel;
                        }

                        cmd_vel.angular = cmd_vel_angular_z_rotate_;
                    }
                }
                else
                {
                    cmd_vel.angular = cmd_vel_angular_z_rotate_;
                }
            }
            logInfo << "FTCPlanner: cmd_vel.z: "<<cmd_vel.angular<<", angle: "<< angle;
            return true;
        }
        else
        {
            cmd_vel_angular_z_rotate_ = 0;
            cmd_vel.angular = 0;
            return false;
        }
    }

    int FTCPlanner::driveToward(sgbot::Pose2D current_pose, sgbot::Velocity2D& cmd_vel)
    {
        float distance = 0;
        float angle = 0;
        int max_point = 0;

        //Search for max achievable point on global plan.
        max_point = checkMaxDistance(current_pose);
        max_point = checkMaxAngle(max_point, current_pose);
        logInfo <<"drvie toward max point = "<<max_point;

        double cmd_vel_linear_x_old = cmd_vel_linear_x_;
        double cmd_vel_angular_z_old = cmd_vel_angular_z_;

        sgbot::Pose2D x_pose;
        x_pose = global_plan_.at(max_point);

        distance = sgbot::distance(x_pose,current_pose);
        angle = calculateGlobalPlanAngle(current_pose, global_plan_, max_point);

        //check if max velocity is exceeded
        if((distance/sim_time) > max_x_vel)
        {
            cmd_vel_linear_x_ = max_x_vel;
        }
        else
        {
            cmd_vel_linear_x_ = (distance/sim_time);
        }

        //check if max rotation velocity is exceeded
        if(fabs(angle/sim_time)>max_rotation_vel)
        {
            cmd_vel_angular_z_ = max_rotation_vel;
        }
        else
        {
            cmd_vel_angular_z_ = (angle/sim_time);
        }

        //Calculate new velocity with max acceleration
        if(cmd_vel_linear_x_ > cmd_vel_linear_x_old+acceleration_x/local_planner_frequence)
        {
            cmd_vel_linear_x_ = cmd_vel_linear_x_old+acceleration_x/local_planner_frequence;
        }
        else
        {
            if(cmd_vel_linear_x_ < cmd_vel_linear_x_old-acceleration_x/local_planner_frequence)
            {
                cmd_vel_linear_x_ = cmd_vel_linear_x_old-acceleration_x/local_planner_frequence;
            }
            else
            {
                cmd_vel_linear_x_ = cmd_vel_linear_x_old;
            }
        }

        //Calculate new velocity with max acceleration
        if(fabs(cmd_vel_angular_z_) > fabs(cmd_vel_angular_z_old)+fabs(acceleration_z/local_planner_frequence))
        {
            if(cmd_vel_angular_z_ < 0)
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old-acceleration_z/local_planner_frequence;
            }
            else
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old+acceleration_z/local_planner_frequence;
            }
        }

        if(cmd_vel_angular_z_ < 0 && cmd_vel_angular_z_old > 0)
        {
            if( fabs(cmd_vel_angular_z_ - cmd_vel_angular_z_old) > fabs(acceleration_z/local_planner_frequence))
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old - acceleration_z/local_planner_frequence;
            }
        }

        if(cmd_vel_angular_z_ > 0 && cmd_vel_angular_z_old < 0)
        {
            if( fabs(cmd_vel_angular_z_ - cmd_vel_angular_z_old) > fabs(acceleration_z/local_planner_frequence))
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old + acceleration_z/local_planner_frequence;
            }
        }

        //Check at last if velocity is to high.
        if(cmd_vel_angular_z_ > max_rotation_vel)
        {
            cmd_vel_angular_z_ = max_rotation_vel;
        }
        if(cmd_vel_angular_z_ < -max_rotation_vel)
        {
            cmd_vel_angular_z_ = (- max_rotation_vel);
        }
        if(cmd_vel_linear_x_ >  max_x_vel)
        {
            cmd_vel_linear_x_ = max_x_vel;
        }
        //Push velocity to cmd_vel for driving.
        cmd_vel.linear = cmd_vel_linear_x_;
        cmd_vel.angular = cmd_vel_angular_z_;
        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_;
        logInfo << "FTCPlanner: max_point: "<<max_point<<", distance: "<<distance<<", x_vel: "<<cmd_vel.linear<<", rot_vel: "<<cmd_vel.angular<<", angle: "<<angle;

        return max_point;
    }


    bool FTCPlanner::isGoalReached()
    {
        if(goal_reached_)
        {
            logInfo << ("FTCPlanner: Goal reached.");
        }
        return goal_reached_;
    }

    bool FTCPlanner::checkCollision(int max_points)
    {
        //maximal costs
        unsigned char previous_cost = 255;

        for (int i = 0; i <= max_points; i++)
        {
            sgbot::Pose2D x_pose;
            x_pose = global_plan_.at(i);

            unsigned int x;
            unsigned int y;
            costmap->getCostmap()->worldToMap(x_pose.x(), x_pose.y(), x, y);
            unsigned char costs = costmap->getCostmap()->getCost(x, y);
            //Near at obstacle
            if(costs > 0)
            {
                if(!rotate_to_global_plan_)
                {
                    logInfo << ("FTCPlanner: Obstacle detected. Start routine new.");
                }
                rotate_to_global_plan_ = true;

                //Possible collision
                if(costs > 127 && costs > previous_cost)
                {
                    logInfo << ("FTCPlanner: Possible collision. Stop local planner.");
                    return false;
                }
            }
            previous_cost = costs;
        }
        return true;
    }

    void FTCPlanner::publishPlan(int max_point)
    {

        FILE* file = fopen("/tmp/ftc_local_plan.log","w+");
        for(int i = 0;i < max_point;++i){
        	fprintf(file,"%d %d\n",global_plan_[i].x(),global_plan_[i].y());
        }
        delete file;
    }

    FTCPlanner::~FTCPlanner()
    {
    }
}
