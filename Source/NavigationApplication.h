/*
 * NavigationApplication.h
 *
 *  Created on: Jan 10, 2018
 *      Author: pengjiawei
 */

#ifndef NAVIGATIONAPPLICATION_H_
#define NAVIGATIONAPPLICATION_H_
#include <Application/Application.h>
#include <transform/transform2d.h>
#include "planner/base/GlobalPlannerBase.h"
#include "planner/base/LocalPlannerBase.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <DataSet/Publisher.h>
#include <DataSet/Subscriber.h>
#include <Service/Server.h>
#include <Service/Client.h>
#include <Mission/Executor.h>
namespace NS_Navigation {

enum NaviState {
	PLANNING, CONTROLLING, CLEARING, WALKING
};
enum S_STATE{
	LEFT = 1, ONE_STEP, RIGHT, RUN
};
enum ACTION : int{
	SEARCH_WALL = 0,
	GOT_WALL,
	ALONG_WALL,
	MASTER_PLAN_POSE,
	MASTER_CONTROL_VELOCITY
};
enum EVENT : int{
	IDLE = 0,
	KNOCK_RELEASE,
	KNOCK_LEFT,
	KNOCK_RIGHT,
	KNOCK_CENTER,
	TOO_NEAR
};

#define PLANNER_LOOP_TIMEOUT 100
/**
 *关于导航功能的类
 */
class NavigationApplication: public Application {
public:
	NavigationApplication();
	virtual ~NavigationApplication();

	void control_func();

	virtual void
	run();
	virtual void
	quit();
private:
	/**
	 * 从文件加载参数
	 */
	void loadParameters();

	/**
	 * 全局规划器计算路径
	 */
	bool
	makePlan(const sgbot::Pose2D& goal, std::vector<sgbot::Pose2D>& plan);

	void
	planLoop();

	void
	controlLoop();

	void listenLoop();
	bool goal_callback(sgbot::Pose2D& goal_from_app);
	//TODO not sure how to implement this
	void visualizedGlobalGoal(sgbot::Pose2D& global_goal_);
	//TODO not sure how to implement this
	void visualizedCurrentPose(sgbot::Pose2D& current_pose);
	//TODO not sure how to implement this
	void visualizedPlan(std::vector<sgbot::Pose2D>& plan_);
	/**
	 * transform the goal in robot frame to global frame
	 * @param goal,pose in robot frame
	 * @return pose in global frame
	 */
	sgbot::Pose2D
	goalToGlobalFrame(sgbot::Pose2D& goal);
	/**
	 * to stop the robot
	 */
	void
	publishZeroVelocity();
	/**
	 * make the robot move
	 */
	void
	publishVelocity(double linear_x, double linear_y, double angular_z);

	/**
	 * recovery behavior ,it's nothing by now
	 */
	void
	runRecovery();
	/**
	 * reset the control state
	 */
	void
	resetState();

	void event_callback(int event_flag);

	void action_callback(int action_flag);
	void turnleft() {
		logInfo<<"turn left";
//	    sgbot::Pose2D pose2d;
//	    if(pose_cli->call(pose2d)){
//
//	    }else{
//	    	logInfo << "call pose 2d failed";
//	    }
		///at base frame
		sgbot::Pose2D pose2d(0,0,M_PI_2);
		goal = pose2d;
		state = PLANNING;
		runPlanner_ = true;
		planner_cond.notify_one();
	}

	void turnright() {
		logInfo<<"turn right";
		sgbot::Pose2D pose2d(0,0,-M_PI_2);
		goal = pose2d;
		state = PLANNING;
		runPlanner_ = true;
		planner_cond.notify_one();
	}

	void GoAhead(double distance) {
		//qian +x , zuo +y
		logInfo<<"go ahead distance = "<<distance;
			sgbot::Pose2D pose2d(distance,0,0);
			goal = pose2d;
			state = PLANNING;
			runPlanner_ = true;
			planner_cond.notify_one();
	}

	void oneStep() {
		logInfo<<("one step");
		//it should be the radius of robot
		float resolution = global_costmap->getLayeredCostmap()->getCostmap()->getResolution();
		logInfo << "resolution = "<< resolution;
		double step_size = resolution;
		GoAhead(step_size);
	}

	void Run() {
		logInfo<<("run");
		float run_distance = 5.f;
		GoAhead(run_distance);
	}

private:
	std::string global_planner_type_;
	std::string local_planner_type_;
	/// 全局规划频率
	float planner_frequency_;
	/// 控制频率
	float controller_frequency_;

	float listen_frequency;
//	/// 小车内切半径
//	double inscribed_radius_;
//	/// 外切半径
//	double circumscribed_radius_;

	float back_to_begin_tolerance;
	float planner_patience_, controller_patience_;
	float oscillation_timeout_, oscillation_distance_;
private:
	std::vector<sgbot::Pose2D>* global_planner_plan;
	std::vector<sgbot::Pose2D>* latest_plan;

	sgbot::Pose2D oscillation_pose_;

	NS_CostMap::CostmapWrapper* global_costmap;

	NS_CostMap::CostmapWrapper* local_costmap;

	NS_Planner::GlobalPlannerBase* global_planner;

	NS_Planner::LocalPlannerBase* local_planner;

	sgbot::Pose2D goal;

	bool new_goal_trigger;
	/// for local planner set plan
	bool new_global_plan_;
	///run plan thread or not
	bool runPlanner_;

	boost::thread plan_thread;
	boost::mutex planner_mutex;
	boost::condition planner_cond;

	boost::thread control_thread;
	boost::mutex controller_mutex;
	boost::condition controller_cond;

	boost::thread listen_thread;

	NaviState state;

	///publish velocity to controller
	NS_DataSet::Publisher<Velocity2D>* twist_pub;
	///subscribe the goal from other
	NS_DataSet::Subscriber<sgbot::Pose2D>* goal_sub;
	///event from controller
	NS_DataSet::Subscriber<int>* event_sub;
	///action pub and sub to controller
	NS_DataSet::Publisher<int>* action_pub;
	NS_DataSet::Subscriber<int>* action_sub;

	///publisher goal
	NS_DataSet::Publisher<sgbot::Pose2D>* goal_pub;
	///client call  pose
	NS_Service::Client<sgbot::Pose2D>* pose_cli;
	///global goal for visualized
	NS_Service::Server<sgbot::Pose2D>* global_goal_srv;
	sgbot::Pose2D global_goal;
	///current pose for visualized
	NS_Service::Server<sgbot::Pose2D>* current_pose_srv;
	sgbot::Pose2D current_pose;
	///plan for visualized
	NS_Service::Server<std::vector<sgbot::Pose2D> >* plan_srv;
	std::vector<sgbot::Pose2D> global_plan;

	///TODO not sure how to do this
	NS_DataSet::Publisher<bool>* explore_pub;
	///mission executor
	NS_Mission::Executor* goalCallbackExecutor;
	///state for walking,default 3
	int current_state;
	int first_trigger = 1;
	//record first trigger too near pose
	sgbot::Pose2D first_pose;
	///
	bool is_walking;

	int action_flag_;
	int state_array[8] = { RIGHT, ONE_STEP, RIGHT, RUN, LEFT, ONE_STEP, LEFT, RUN };
};

}
/* namespace NS_Navigation */

#endif /* NAVIGATIONAPPLICATION_H_ */
