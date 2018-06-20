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
#include "planner/implements/FTCLocalPlanner/ftc_planner.h"
#include <type/map2d.h>
namespace NS_Navigation {

NavigationApplication::NavigationApplication() :
		new_global_plan_(false), runPlanner_(false) {
	// TODO Auto-generated constructor stub
	goal_sub = new NS_DataSet::Subscriber<sgbot::Pose2D>("GOAL_FROM_APP",
			boost::bind(&NavigationApplication::goal_callback, this, _1));
	mapping_sub = new NS_DataSet::Subscriber<int>("MAP_READY",
			boost::bind(&NavigationApplication::mappingCallback, this, _1));
//	event_sub = new NS_DataSet::Subscriber<int>("SLAVE_EVENT",
//			boost::bind(&NavigationApplication::eventCallback, this, _1));
//	action_sub = new NS_DataSet::Subscriber<int>("SLAVE_ACTION",
//			boost::bind(&NavigationApplication::actionCallback, this, _1));
	action_pub = new NS_DataSet::Publisher<int>("MASTER_ACTION");
	pose_cli = new NS_Service::Client<sgbot::Pose2D>("POSE");

	twist_pub = new NS_DataSet::Publisher<sgbot::Velocity2D>("TWIST");

	pose_theta_pub = new NS_DataSet::Publisher<float>("BORDER_THETA");
	pose_distance_pub = new NS_DataSet::Publisher<float>("BORDER_DIST");
	coverage_pub = new NS_DataSet::Publisher<int>("COVERAGE");
}

NavigationApplication::~NavigationApplication() {
	// TODO Auto-generated destructor stub
	delete goal_sub;
	delete goal_pub;
	delete pose_cli;
	delete event_sub;
	delete action_sub;
	delete action_pub;
	delete twist_pub;
	delete pose_theta_pub;
	delete pose_distance_pub;
	delete coverage_pub;
}
void NavigationApplication::loadParameters() {
	NS_NaviCommon::Parameter parameter;

	parameter.loadConfigurationFile("navigation.xml");
	global_planner_type_ = parameter.getParameter("global_planner_type",
			"global_planner");
	local_planner_type_ = parameter.getParameter("local_planner_type",
			"ftc_local_planner");
	planner_frequency_ = parameter.getParameter("planner_frequency", 1.0f);
	controller_frequency_ = parameter.getParameter("controller_frequency",
			5.0f);
	back_to_begin_tolerance = parameter.getParameter("back_to_begin_tolerance",
			0.1f);
	listen_frequency = parameter.getParameter("listen_frequency", 1.f);
	one_step = parameter.getParameter("one_step", 0.25f);
	run_distance = parameter.getParameter("run_distance", 2.f);
	if (parameter.getParameter("is_log_file", 0) == 1) {
		is_log_file = true;
	} else {
		is_log_file = false;
	}
	//simple_turn
	if (parameter.getParameter("simple_turn", 1) == 1) {
		simple_turn = true;
	} else {
		simple_turn = false;
	}
	simple_turn_vel = parameter.getParameter("simple_turn_vel", 0.3f);
	simple_turn_tolerance = parameter.getParameter("simple_turn_tolerance",
			0.2f);

}
void NavigationApplication::runRecovery() {
	logInfo<<"run Recovery()";
}

void NavigationApplication::resetState() {
	state = PLANNING;
	publishZeroVelocity();
}
void NavigationApplication::publishZeroVelocity() {
	logInfo<<"--------------->publishZeroVelocity------------------------>";
	publishVelocity(0, 0, 0);
}
void NavigationApplication::publishVelocity(double linear_x, double linear_y,
		double angular_z) {
	Velocity2D vel;
	vel.linear = linear_x;
	vel.angular = angular_z;
//	logInfo<<"--------------->publishVelocity to controller------------------------>"<<"linear_x = "<<linear_x<<" angular = "<<angular_z;
	twist_pub->publish(vel);
}
bool NavigationApplication::goal_callback(sgbot::Pose2D& goal_from_app) {
	planner_mutex.lock();
	goal = goalToGlobalFrame(goal_from_app);
//		goal = goal_from_app;
	printf("goal_callback x = %.4f,y = %.4f, theta = %.4f\n", goal.x(),
			goal.y(), goal.theta());
	state = PLANNING;
	planner_cond.notify_one();
	runPlanner_ = true;
	planner_mutex.unlock();

	return true;
}
sgbot::Pose2D NavigationApplication::goalToGlobalFrame(sgbot::Pose2D& goal) {
	sgbot::Pose2D pose, target_pose;
	if (pose_cli->call(pose)) {
		sgbot::tf::Transform2D map_transform(pose.x(), pose.y(), pose.theta(),
				1);
		target_pose = map_transform.transform(goal);
		logInfo<<"pose 2d in global frame is "<<target_pose.x()<<" "<<target_pose.y()<<" "<<target_pose.theta();
	}
	return target_pose;
}

//void NavigationApplication::actionCallback(int action_flag) {
//	logInfo<< "action callback flag = "<<action_flag;
//	action_flag_ = action_flag;
//}
//void NavigationApplication::eventCallback(int event_flag) {
//	logInfo<< "event callback flag = "<<event_flag;
//	if(event_flag == TOO_NEAR) {
//		if(first_trigger == 1) {
//			logInfo << "first trigger too near so record current pose";
//			if(!pose_cli->call(first_pose)) {
//				logInfo << "first trigger get first pose failed";
//			} else {
//				logInfo << "first pose"<<first_pose.x()<<" , "<<first_pose.y();
//				first_trigger = 0;
//			}
//		}
//		++too_near_count;
//		logInfo << "too near count ++ = "<<too_near_count;
//
//		if(too_near_count >= 2 ) {
//			sgbot::Pose2D pose;
//			if (!pose_cli->call(pose)) {
//				logError<<"call pose failed";
//			}
//			sgbot::Point2D point(pose.x(),pose.y());
//			logInfo << "push back point "<<too_near_count - 1<< " = "<<pose.x()<<" , "<<pose.y();
//			if(too_near_count == 2 && trigger_times == 1) {
//				triggered_pose = pose;
//				logInfo <<"too near count == 2 assign triggered_pose"<<triggered_pose.x()<<" , "<<triggered_pose.y();
//				trigger_times = 0;
//			}
//			point_vec.push_back(point);
//		}
//
//	}
//}

//void NavigationApplication::prepare_for_walk(){
//	logInfo << "prepare for walking,is walking = "<<is_walking<<" corner point[0]  = "<<point_vec[0].x()<<" , "<<point_vec[0].y();
//	is_preparing = 1;
//	goal = sgbot::Pose2D(point_vec[0].x(),point_vec[0].y(),callback_theta);
//	state = PLANNING;
//	runPlanner_ = true;
//	planner_cond.notify_one();
//}

void NavigationApplication::listenLoop() {
	NS_NaviCommon::Rate rate(1.f);
	while (running) {
		backToWalkS();
		findFrontWall();
		wolkSComplete();
		rate.sleep();
	}
}
void NavigationApplication::mappingCallback(int flag){
	logInfo <<"start mapping so start listening loop thread";
	listen_thread = boost::thread(
				boost::bind(&NavigationApplication::listenLoop, this));
}
void NavigationApplication::searchGoWall() {
	std::vector<boost::shared_ptr<NS_CostMap::CostmapLayer> >* layer_vec_p =
			global_costmap->getLayeredCostmap()->getPlugins();
	boost::shared_ptr<NS_CostMap::CostmapLayer> costmap_layer = layer_vec_p->at(
			2);
	boost::shared_ptr<NS_CostMap::VisitedLayer> visited_layer =
			boost::dynamic_pointer_cast<NS_CostMap::VisitedLayer>(
					costmap_layer);
	unsigned int first_frontier_index = visited_layer->searchWallPoint();
	unsigned int x1, y1;
	global_costmap->getCostmap()->indexToCells(first_frontier_index, x1, y1);
	float world_x, world_y;
	global_costmap->getCostmap()->mapToWorld(x1, y1, world_x, world_y);
	first_pose.x() = world_x;
	first_pose.y() = world_y;
	logInfo<< " searched wall = "<<world_x<<" , "<<world_y;
	sgbot::Pose2D pose;
	if (pose_cli->call(pose)) {
		float theta = sgbot::math::atan2(world_y - pose.y(),
				world_x - pose.x());
		logInfo<< " publish theta = "<<theta;
		pose_theta_pub->publish(theta);
		///go to wall
		int action = GOTO_WALL;
		action_pub->publish(action);
	}

}
void NavigationApplication::triggerLoopTurn() {
	std::vector<boost::shared_ptr<NS_CostMap::CostmapLayer> >* layer_vec_p =
			global_costmap->getLayeredCostmap()->getPlugins();
	boost::shared_ptr<NS_CostMap::CostmapLayer> costmap_layer = layer_vec_p->at(
			2);
	boost::shared_ptr<NS_CostMap::VisitedLayer> visited_layer =
			boost::dynamic_pointer_cast<NS_CostMap::VisitedLayer>(
					costmap_layer);

	sgbot::Pose2D pose;
	if (pose_cli->call(pose)) {
		if (current_rect->isNearCorner(pose) && global_state == CIRCLE) {
			logInfo<< "current pose is near corner trigger loop turn";
			float theta = M_PI / 2;
			pose_theta_pub->publish(theta);
		}
	}

}
void NavigationApplication::backToWalkS() {
	sgbot::Pose2D pose;
	if (pose_cli->call(pose) && global_state == CIRCLE) {
		if (sgbot::distance(first_pose, pose) < back_to_begin_tolerance) {
			int action = WALK_S_PATH; ///from loop to walk s,action = spath
			logInfo<< "pub action walk s path and start coverage";
			global_state = WALK_S;
			action_pub->publish(action);
			int start_coverage = 1;
			coverage_pub->publish(start_coverage);
		}
	}

}
void NavigationApplication::findFrontWall() {
	sgbot::Pose2D pose;
	if (pose_cli->call(pose) && global_state == WALK_S) {
		boost::shared_ptr<NS_CostMap::VisitedLayer> visited_layer =
				get_visited_layer();
		std::vector<int> walls = visited_layer->searchFrontWall();
		int start_x = walls[0]
				% global_costmap->getCostmap()->getSizeInCellsX();
		int start_y = walls[0]
				/ global_costmap->getCostmap()->getSizeInCellsX();

		int end_x = walls.back()
				% global_costmap->getCostmap()->getSizeInCellsX();
		int end_y = walls.back()
				/ global_costmap->getCostmap()->getSizeInCellsX();

		float world_x, world_y;
		global_costmap->getCostmap()->mapToWorld(start_x, start_y, world_x,
				world_y);
		float distance = sgbot::distance(pose,
				sgbot::Pose2D(world_x, world_y, 0.f));
		logInfo<< "front wall distance = "<<distance;
		if (distance < 0.3f) {
			float theta = std::atan2(end_y - start_y, end_x - start_x);
			logInfo<<"found frontier wall theta = "<<theta;
			pose_theta_pub->publish(theta);
			pose_distance_pub->publish(distance);
		}

	}
}
//need to search another area uncovered
void NavigationApplication::fullCoverage() {
	if (global_state == CIRCLE) {
		boost::shared_ptr<NS_CostMap::VisitedLayer> visited_layer =
				get_visited_layer();
		sgbot::Pose2D pose;
		pose_cli->call(pose);
		int result_idx;
		visited_layer->nearestCell(result_idx, pose);
		int index_x = result_idx % global_costmap->getCostmap()->getSizeInCellsX();
		int index_y = result_idx / global_costmap->getCostmap()->getSizeInCellsX();
		logInfo<< "full coveraged search new area index x = "<<index_x <<" , "<<index_y;
		logInfo<<"and now run planner";
		float world_x,world_y;
		global_costmap->getCostmap()->mapToWorld(index_x,index_y,world_x,world_y);
		sgbot::Pose2D pose2d(world_x, world_y,0.f);
		goal = pose2d;
		state = PLANNING;
		runPlanner_ = true;
		planner_cond.notify_one();
	}
}
void NavigationApplication::wolkSComplete() {
	if (global_state == CIRCLE) {
		boost::shared_ptr<NS_CostMap::VisitedLayer> visited_layer =
				get_visited_layer();
		sgbot::Pose2D pose;
		pose_cli->call(pose);
		unsigned int map_x, map_y;
		global_costmap->getCostmap()->worldToMap(pose.x(), pose.y(), map_x,
				map_y);
		int index = map_x
				+ map_y * global_costmap->getCostmap()->getSizeInMetersX();
		std::vector<int> nei_8 = visited_layer->neighborhood8(pose, index);
		int covered_count = 0;
		for (auto value : nei_8) {
			unsigned int map_x, map_y;
			global_costmap->getCostmap()->indexToCells(value, map_x, map_y);
			if (visited_layer->isCovered(map_x, map_y)) {
				++covered_count;
				if (covered_count >= 3) {
					break;
				}
			}
		}
		logInfo<< "covered complete current pose = "<<pose.x()<<" , "<<pose.y();
	}
	fullCoverage();
}

void NavigationApplication::planLoop() {
	NS_NaviCommon::Rate rate(planner_frequency_);

	while (running) {
		planner_mutex.lock();
		while (new_goal_trigger && running && !runPlanner_) {
			logInfo<< "planner_condition waiting all long ";
			planner_cond.wait(planner_mutex);
			new_goal_trigger = false;
		}
		planner_mutex.unlock();

		if (!running) {
			console.message("Quit global planning loop...");
			break;
		}
		//&& state != PLANNING
		if (!makePlan(goal, *latest_plan)) {
			console.error("Make plan failure!");
//			goalCallbackExecutor->abort();
//			is_walking = 0;
			new_goal_trigger = true;
			logInfo<< "global planner failed to make plan , maybe recovery should be triggered";
			continue;
		}

		controller_mutex.lock();
		state = CONTROLLING;
		new_global_plan_ = true;
		global_planner_plan->clear();
		global_planner_plan->assign(latest_plan->begin(), latest_plan->end());
		if (runPlanner_)
			state = CONTROLLING;
		controller_cond.notify_one();
		controller_mutex.unlock();

		if (planner_frequency_ <= 0.0f)
			runPlanner_ = false;
		else
			rate.sleep();
		new_goal_trigger = true;
		logInfo<< "make plan done once";
	}
}

void NavigationApplication::controlLoop() {
	while (running) {
		logInfo<< "control loop state ="<<state;
		NS_NaviCommon::Rate rate(controller_frequency_);
		//TODO maybe controller_frequency should be used
		controller_mutex.lock();
		while ((state != CONTROLLING && state != WALKING)
				&& running) {
			logInfo<< "controller_condition waiting all long state = "<<state;
			controller_cond.wait(controller_mutex);
		}
		controller_mutex.unlock();

		if (!running) {
			console.message("Quit local planning loop...");
			break;
		}
		if (new_global_plan_) {
			new_global_plan_ = false;
			if (!local_planner->setPlan(*global_planner_plan)) {
				console.error("Set plan to local planner failure!");
				resetState();
				logInfo<< "local planner failed tp set plan,so disable plan thread ";
				planner_mutex.lock();
				runPlanner_ = false;
				planner_mutex.unlock();
				continue;
			}
		} else {
			logInfo<< "no new global plan do not set plan";
		}
		//update feedback to correspond to our curent position
		sgbot::Pose2D global_pose;
		global_costmap->getRobotPose(global_pose);

		printf("global_pose x = %.4f,y = %.4f, w = %.4f ,state = %d\n",
				global_pose.x(), global_pose.y(), global_pose.theta(), state);

		sgbot::Pose2D current_position;
		current_position = global_pose;

		if (sgbot::distance(current_position, oscillation_pose_)
				>= oscillation_distance_) {
			//TODO: oscillation
			logInfo<< "oscillation is triggered , but do nothing now";
			oscillation_pose_ = current_position;
		}
		switch (state) {
			case PLANNING:
			logInfo<< "control loop state is planning";
			planner_mutex.lock();
			planner_cond.notify_one();
			runPlanner_ = true;
			planner_mutex.unlock();
			break;
			case CONTROLLING:
			controlFunc();
			break;
			case CLEARING:
			runRecovery();
			state = PLANNING;
			break;
			case WALKING:
			switch (state_array[current_state]) {
				case LEFT:
				turnleft();
				break;
				case ONE_STEP:
				oneStep();
				break;
				case RIGHT:
				turnright();
				break;
				case RUN:
				Run();
				break;
				default:
				logInfo << "Switch control nothing";
				break;
				break;
			}
		}
		rate.sleep();
	}
}

void NavigationApplication::controlFunc() {
	logInfo<< "control func hold until control finished";
//	while(running) {
	Velocity2D cmd_vel;
	NS_NaviCommon::Time last_valid_control;
	if (local_planner->isGoalReached()) {
		console.message("The goal has reached!");
//			goalCallbackExecutor->done();
		//             printf("continue exploring? = %d\n", isExploring);
		//             publishIsExploring();
		resetState();
		planner_mutex.lock();
		runPlanner_ = false;
		planner_mutex.unlock();
//			if(is_preparing){
//				is_ready_for_walk = 1;
//				is_preparing = 0;
//			}
//			if(is_walking) {
//				logInfo << "goal reached continue walking";
//				current_state = (++current_state) % 8;
//				state = WALKING;
//			}
		return;
	}

	//TODO : check oscillation and clear
	clock_t start, end;
	start = clock();

	if (local_planner->computeVelocityCommands(cmd_vel)) {
		console.debug("Got velocity data : linear=%.3lf, angular=0.0f!",
				cmd_vel.linear, cmd_vel.angular);
		last_valid_control = NS_NaviCommon::Time::now();
		publishVelocity(cmd_vel.linear, 0.0, cmd_vel.angular);
		end = clock();
	} else {
		console.warning("The planner can not got a valid velocity data!");
		NS_NaviCommon::Time next_control = last_valid_control
		+ NS_NaviCommon::Duration(controller_patience_);

		if (NS_NaviCommon::Time::now() > next_control) {
			publishZeroVelocity();
			state = CLEARING;
			logInfo<< "can't get valid vel and out of control patience , recovery behavior should be triggered";
			return;
		}
		else
		{
			//TODO: re-plan

			logInfo << "go back to planning";
			publishZeroVelocity();
			state = PLANNING;

			planner_mutex.lock();
			runPlanner_ = true;
			planner_cond.notify_one();
			planner_mutex.unlock();
			return;
		}
	}
//	}
}

bool NavigationApplication::makePlan(const sgbot::Pose2D& goal,
		std::vector<sgbot::Pose2D>& plan) {

	boost::unique_lock<NS_CostMap::Costmap2D::mutex_t> lock(
			*(global_costmap->getLayeredCostmap()->getCostmap()->getMutex()));

	plan.clear();

	//get the starting pose of the robot
	sgbot::Pose2D global_pose;
	if (!global_costmap->getRobotPose(global_pose)) {
		console.warning(
				"Unable to get starting pose of robot, so make plan as robot pose is 0,0,0");
	}

	sgbot::Pose2D start = global_pose;

	//if the planner fails or returns a zero length plan, planning failed
	if (!global_planner->makePlan(start, goal, plan) || plan.empty()) {
		console.warning("Failed to find a  plan to point (%.2f, %.2f)",
				goal.x(), goal.y());
		return false;
	}

	logInfo<< "Plans computed,  points to go... = " << plan.size();
	global_plan.resize(plan.size());
	for (size_t i = 0; i < plan.size(); i++) {
		global_plan[i] = plan[i];
		printf("%lf,%lf,\n", plan[i].x(), plan[i].y());
	}
	logInfo<< "global_plan is assigned and size  = "<< global_plan.size();
	return true;

}

void NavigationApplication::run() {
	loadParameters();
	std::string file_pp = "/tmp/";
	if (is_log_file) {
		addFileLog(file_pp);
	}
	//set up plan triple buffer
	global_planner_plan = new std::vector<sgbot::Pose2D>();
	latest_plan = new std::vector<sgbot::Pose2D>();

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

	if (local_planner_type_ == "trajectory_local_planner") {
		local_planner = new NS_Planner::TrajectoryLocalPlanner();
	} else {
		local_planner = new NS_Planner::FTCPlanner();
	}

	local_planner->initialize(global_costmap);

	sleep(2);
	logInfo<< "initial state to planning and wait for listening thread to set walking";
	state = PLANNING;

	new_goal_trigger = true;

	NS_Service::Client<sgbot::Map2D> map_cli("MAP");
	for (int i = 0; i < 10; i++) {
		sgbot::Map2D map;
		if (map_cli.call(map)) {
			break;
		}
	}

	running = true;

	plan_thread = boost::thread(
			boost::bind(&NavigationApplication::planLoop, this));

	control_thread = boost::thread(
			boost::bind(&NavigationApplication::controlLoop, this));

	listen_thread = boost::thread(
			boost::bind(&NavigationApplication::listenLoop, this));
	global_costmap->start();

	global_state = CIRCLE;
//	logInfo<< "search wall";
	current_state = 0;
//	int action = SEARCH_WALL;
//	action_flag_ = action;
//	action_pub->publish(action);

///below are test for debug
//	sleep(5);
//	state = WALKING;
//	is_walking = 1;
//	controller_mutex.lock();
//	controller_cond.notify_one();
//	controller_mutex.unlock();
//	callback_theta = 0.f;

//	backToWalkS();
//		findFrontWall();
//		wolkSComplete();

	//generate rectangle
	logInfo<< "generate first rectangle";
	std::vector<boost::shared_ptr<NS_CostMap::CostmapLayer> >* layer_vec_p =
			global_costmap->getLayeredCostmap()->getPlugins();
	boost::shared_ptr<NS_CostMap::CostmapLayer> costmap_layer = layer_vec_p->at(
			2);
	boost::shared_ptr<NS_CostMap::VisitedLayer> visited_layer =
			boost::dynamic_pointer_cast<NS_CostMap::VisitedLayer>(
					costmap_layer);

	sgbot::Pose2D pose;
	pose_cli->call(pose);
	current_rect = visited_layer->generateRectangle(pose);

}

void NavigationApplication::quit() {
	console.message("navigation is quitting!");

	running = false;
	plan_thread.join();
	listen_thread.join();
	control_thread.join();
}
} /* namespace NS_Navigation */
