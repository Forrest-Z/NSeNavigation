/*
 * NavigationApplication.cpp
 *
 *  Created on: Jan 10, 2018
 *      Author: pengjiawei
 */

#include "NavigationApplication.h"
#include <Parameter/Parameter.h>
#include "planner/implements/GlobalPlanner/GlobalPlanner.h"
#include "planner/implements/TrajectoryLocalPlanner/TrajectoryLocalPlanner.h"
#include <Service/ServiceType/ServiceMap.h>
namespace NS_Navigation {

NavigationApplication::NavigationApplication() {
	// TODO Auto-generated constructor stub

}

NavigationApplication::~NavigationApplication() {
	// TODO Auto-generated destructor stub
}
void NavigationApplication::loadParameters() {
	NS_NaviCommon::Parameter parameter;

	parameter.loadConfigurationFile("navigation.xml");
	global_planner_type_ = parameter.getParameter("global_planner_type",
			"global_planner");
	local_planner_type_ = parameter.getParameter("local_planner_type",
			"trajectory_local_planner");
	planner_frequency_ = parameter.getParameter("planner_frequency", 0.0f);
	controller_frequency_ = parameter.getParameter("controller_frequency",
			5.0f);
}
void NavigationApplication::runRecovery()
 {

 }

void NavigationApplication::resetState()
 {
   state = PLANNING;
   publishZeroVelocity();
 }
void NavigationApplication::publishZeroVelocity()
{
  printf("--------------->publishZeroVelocity------------------------>");
  publishVelocity(0, 0, 0);
}
void NavigationApplication::publishVelocity(double linear_x, double linear_y,
                                            double angular_z)
{
  NS_DataType::Twist vel;
  vel.linear.x = linear_x;
  vel.linear.y = linear_y;
  vel.angular.z = angular_z;
  twist_pub->publish(vel);
}
void NavigationApplication::controlLoop()
 {
   while(running)
   {
     NS_NaviCommon::Rate rate(controller_frequency_);
     controller_mutex.lock();
     while((state != CONTROLLING || global_planner_plan->size() == 0) && running)
     {
       controller_cond.timed_wait(
           controller_mutex,
           (boost::get_system_time() + boost::posix_time::milliseconds(
               PLANNER_LOOP_TIMEOUT)));
     }
     controller_mutex.unlock();

     if(!running)
     {
       console.message("Quit local planning loop...");
       break;
     }
     if(!local_planner->setPlan(*global_planner_plan))
     {
       console.error("Set plan to local planner failure!");
       resetState();
       continue;
     }
     bool is_planning = 0;
     while(!is_planning)
     {
       NS_DataType::Twist cmd_vel;
       NS_NaviCommon::Time last_valid_control;

       //update feedback to correspond to our curent position
       NS_Transform::Stamped < NS_Transform::Pose > global_pose;
       global_costmap->getRobotPose(global_pose);

       printf("global_pose x = %.4f,y = %.4f, w = %.4f ,state = %d\n",
              global_pose.getOrigin().x(), global_pose.getOrigin().y(),
              global_pose.getRotation().w(), state);

       NS_DataType::PoseStamped current_position;
       NS_Transform::poseStampedTFToMsg(global_pose, current_position);

       if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
       {
         //TODO: oscillation

         oscillation_pose_ = current_position;
       }

       switch(state)
       {
         case PLANNING:
           is_planning = 1;
           break;
         case CONTROLLING:
           if(local_planner->isGoalReached())
           {
             console.message("The goal has reached!");
             goalCallbackExecutor->done();
//             printf("continue exploring? = %d\n", isExploring);
//             publishIsExploring();
             resetState();
             break;
           }

           //TODO : check oscillation and clear
           clock_t start, end;
           start = clock();
           FILE* file;

           if(local_planner->computeVelocityCommands(cmd_vel))
           {
             file = fopen("/tmp/vel.log", "a+");
             fprintf(file, "%.3lf,%.3lf,%.3lf\n", cmd_vel.linear.x,
                     cmd_vel.linear.y, cmd_vel.angular.z);
             console.debug(
                 "Got velocity data : l_x=%.3lf, l_y=%.3lf, a_z=%.3lf!",
                 cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
             last_valid_control = NS_NaviCommon::Time::now();
             publishVelocity(cmd_vel.linear.x, cmd_vel.linear.y,
                             cmd_vel.angular.z);
             end = clock();
             fclose(file);
             printf("\n\n\n");
           }
           else
           {
             console.warning("The planner can not got a valid velocity data!");
             NS_NaviCommon::Time next_control = last_valid_control + NS_NaviCommon::Duration(
                 controller_patience_);

             if(NS_NaviCommon::Time::now() > next_control)
             {
               publishZeroVelocity();
               state = CLEARING;
               resetState();
               is_planning = 1;
             }
             else
             {
               //TODO: re-plan

               publishZeroVelocity();
               state = PLANNING;
               is_planning = 1;
//                resetState();
             }
           }

           break;
         case CLEARING:
           runRecovery();
           state = PLANNING;
           is_planning = 1;
           break;
       }

       rate.sleep();
     }
   }
 }


void NavigationApplication::planLoop()
 {
   NS_NaviCommon::Rate rate(planner_frequency_);

   while(running)
   {
     planner_mutex.lock();
     while(!new_goal_trigger && running)
     {
       planner_cond.timed_wait(
           planner_mutex,
           (boost::get_system_time() + boost::posix_time::milliseconds(
               PLANNER_LOOP_TIMEOUT)));
     }
     planner_mutex.unlock();

     if(!running)
     {
       console.message("Quit global planning loop...");
       break;
     }

     new_goal_trigger = false;

     if(!makePlan(goal, *latest_plan) && state != PLANNING)
     {
       console.error("Make plan failure!");
       goalCallbackExecutor->abort();
       continue;
     }

     controller_mutex.lock();
     state = CONTROLLING;
     global_planner_plan->clear();
     global_planner_plan->assign(latest_plan->begin(), latest_plan->end());
     controller_cond.notify_one();
     controller_mutex.unlock();

     if(planner_frequency_ != 0.0f)
       rate.sleep();
   }
 }
NS_DataType::PoseStamped NavigationApplication::goalToGlobalFrame(NS_DataType::PoseStamped& goal){

}
bool NavigationApplication::makePlan(const NS_DataType::PoseStamped& goal,
		std::vector<NS_DataType::PoseStamped>& plan) {

    boost::unique_lock < NS_CostMap::Costmap2D::mutex_t > lock(
        *(global_costmap->getLayeredCostmap()->getCostmap()->getMutex()));

    plan.clear();

    //get the starting pose of the robot
    NS_Transform::Stamped < NS_Transform::Pose > global_pose;
    if(!global_costmap->getRobotPose(global_pose))
    {
      console.error(
          "Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    NS_DataType::PoseStamped start;
    NS_Transform::poseStampedTFToMsg(global_pose, start);

    //if the planner fails or returns a zero length plan, planning failed
    if(!global_planner->makePlan(start, goal, plan) || plan.empty())
    {
      console.warning("Failed to find a  plan to point (%.2f, %.2f)",
                      goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    console.debug("Plans computed, %d points to go...", plan.size());
    global_plan.resize(plan.size());
    for(size_t i = 0; i < plan.size(); i++)
    {
      console.debug("[%d] x = %lf, y = %lf", (i + 1), plan[i].pose.position.x,
                    plan[i].pose.position.y);
      global_plan[i] = plan[i];
      printf("%lf,%lf,\n", plan[i].pose.position.x, plan[i].pose.position.y);
    }
    printf("global_plan is assigned and size = %d\n", global_plan.size());
    return true;

}
double NavigationApplication::distance(const NS_DataType::PoseStamped& p1,
                                       const NS_DataType::PoseStamped& p2)
{
  return hypot(p1.pose.position.x - p2.pose.position.x,
               p1.pose.position.y - p2.pose.position.y);
}
void NavigationApplication::run() {
	loadParameters();

	//set up plan triple buffer
	global_planner_plan = new std::vector<NS_DataType::PoseStamped>();
	latest_plan = new std::vector<NS_DataType::PoseStamped>();

	/*
	 * make global planner and global costmap
	 */
	global_costmap = new NS_CostMap::CostmapWrapper();
	global_costmap->initialize();

	//load global planner
	if (global_planner_type_ == "global_planner") {
		global_planner = new NS_Planner::GlobalPlanner();
	} else {
		global_planner = new NS_Planner::GlobalPlanner();
	}

	global_planner->initialize(global_costmap);

	//load local planner

	local_planner = new NS_Planner::TrajectoryLocalPlanner();

	local_planner->initialize(global_costmap);

	state = PLANNING;

	new_goal_trigger = false;

	NS_Service::Client<NS_ServiceType::ServiceMap> map_cli("MAP");
	for (int i = 0; i < 10; i++) {
		NS_ServiceType::ServiceMap map;
		if (map_cli.call(map)) {
			if (map.result) {
				break;
			}
		}
	}

	running = true;

	plan_thread = boost::thread(
			boost::bind(&NavigationApplication::planLoop, this));

	control_thread = boost::thread(
			boost::bind(&NavigationApplication::controlLoop, this));

	global_costmap->start();
	sleep(2);
//    isExploring = true;
}

void NavigationApplication::quit()
{
  console.message("navigation is quitting!");

  running = false;
  plan_thread.join();
}
} /* namespace NS_Navigation */
