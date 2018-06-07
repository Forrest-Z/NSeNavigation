

#ifndef FTC_LOCAL_PLANNER_FTC_PLANNER_H_
#define FTC_LOCAL_PLANNER_FTC_PLANNER_H_

#include <type/pose2d.h>
#include <log_tool.h>
#include <std-math/math.h>
#include <type/velocity2d.h>
#include "../../base/LocalPlannerBase.h"


namespace NS_Planner
{

    class FTCPlanner : public LocalPlannerBase
    {

    public:
        FTCPlanner();
        /**
         * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        bool computeVelocityCommands(sgbot::Velocity2D& cmd_vel);

        /**
         * @brief  Check if the goal pose has been achieved by the local planner
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        /**
         * @brief  Set the plan that the local planner is following
         * @param plan The plan to pass to the local planner
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<sgbot::Pose2D>& plan);

//        /**
//         * @brief Constructs the local planner
//         * @param name The name to give this instance of the local planner
//         * @param tf A pointer to a transform listener
//         * @param costmap_ros The cost map to use for assigning costs to local plans
//         */
//        void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

        virtual void
        onInitialize();

        ~FTCPlanner();

    private:


        /**
        *@brief Goes along global plan the max distance whith sim_time and max_x_vel allow
        *@param current pose of the robot
        *@return max point of the global plan with can reached
        */
        int checkMaxDistance(sgbot::Pose2D current_pose);

        /**
        *@brief Goes backward along global plan the max angle whith sim_time and max_rotation_vel allow
        *@param point where starts to go backward
        *@param current pose of the robot
        *@return max point of the global plan with can reached
        */
        int checkMaxAngle(int points, sgbot::Pose2D current_pose);

        /**
        *@brief Rotation at place
        *@param angle which is to rotate
        *@param velocity message which is calculate for rotation
        *@param accuracy of orientation
        *@return true if rotate, false if rotation goal reached
        */
        bool rotateToOrientation(float angle, sgbot::Velocity2D& cmd_vel, float accuracy);

        /**
        *@brief Publish the global plan for visulatation.
        *@param points where jused to calculate plan.
        */
        void publishPlan(int max_point);

        /**
        *@brief Drive along the global plan and calculate the velocity
        *@param current pose of the robot
        *@param velocity message
        *@return number of points of global plan which are used
        */
        int driveToward(sgbot::Pose2D current_pose, sgbot::Velocity2D& cmd_vel);

        /**
        *@brief Calculate the orientation of the global plan
        *@param current robot pose
        *@param global plan
        *@param number of points which used for calculation
        */
        float calculateGlobalPlanAngle(sgbot::Pose2D current_pose, const std::vector<sgbot::Pose2D>& plan, int points);

        /**
        *@brief Check if the considerd points are in local collision.
        *@param points of global plan which are considerd.
        *@return true if no collision.
        */
        bool checkCollision(int max_points);

        bool getPoseInPlan(const std::vector<sgbot::Pose2D>& global_plan,sgbot::Pose2D& goal_pose,int plan_point);

        ///result + theta_a = theta_b , and -pi <= result <= pi
        float angleDiff(float theta_a,float theta_b){
        	float angle = theta_b - theta_a;
        	float a = sgbot::math::fmod(sgbot::math::fmod(angle, 2.0 * M_PI) + 2.0 * M_PI , 2.0 * M_PI);
        	if(a > M_PI){
        		a -= 2.0*M_PI;
        	}
        	return a;
        }
        //global plan which we run along
        std::vector<sgbot::Pose2D> global_plan_;

        //check if plan first at first time
        bool first_setPlan_;
        //last point of the global plan in global frame
        sgbot::Pose2D goal_pose_;
        // true if the robot should rotate to gobal plan if new global goal set
        sgbot::Pose2D old_goal_pose_;
        // true if the robot should rotate to gobal plan if new global goal set
        bool rotate_to_global_plan_;
        //true if the goal point is reache and orientation of goal is reached
        bool goal_reached_;
        //true if the goal point is reache and orientation of goal isn't reached
        bool stand_at_goal_;

        //rotation velocity of previous round for the rotateToOrientation methode
        float cmd_vel_angular_z_rotate_;
        //x velocity of the previous round
        float cmd_vel_linear_x_;
        //rotation velocity of previous round for the dirveToward methode
        float cmd_vel_angular_z_;


        //blow are parameters
        float position_accuracy,rotation_accuracy;
        float max_x_vel,sim_time;
        float max_rotation_vel,min_rotation_vel;
        float acceleration_x,acceleration_z;
        float slow_down_factor;
        float local_planner_frequence;
    };
};
#endif
