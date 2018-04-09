/*
 * GlobalPlanner.cpp
 *
 *  Created on: 2016年12月2日
 *      Author: seeing
 */

#include "GlobalPlanner.h"

#include "Algorithm/QuadraticCalculator.h"
#include "Algorithm/GradientPath.h"
#include "Algorithm/Dijkstra.h"
#include <Parameter/Parameter.h>
#include <Console/Console.h>

#include <iostream>
using namespace std;

/*
 * costmap 是用一维数组表示二维，通过 toIndex 来把二维数组中的坐标转化为一维数组中的位置
 */
namespace NS_Planner {

GlobalPlanner::GlobalPlanner() {
}

GlobalPlanner::~GlobalPlanner() {
}

/*
 * 把 costmap 代表的 4 个边上的点全部设为 value 值
 */
void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny,
		unsigned char value) {
	unsigned char* pc = costarr;
	for (int i = 0; i < nx; i++)
		*pc++ = value;
	pc = costarr + (ny - 1) * nx;
	for (int i = 0; i < nx; i++)
		*pc++ = value;
	pc = costarr;
	for (int i = 0; i < ny; i++, pc += nx)
		*pc = value;
	pc = costarr + nx - 1;
	for (int i = 0; i < ny; i++, pc += nx)
		*pc = value;
}

void GlobalPlanner::onInitialize() {
	if (!initialized_) {
		/*
		 * 获取 costmap 的 size_x 和 size_y
		 */
		unsigned int cx =
				costmap->getLayeredCostmap()->getCostmap()->getSizeInCellsX(),
				cy =
						costmap->getLayeredCostmap()->getCostmap()->getSizeInCellsY();
		convert_offset_ = 0.5;
		NS_NaviCommon::Parameter parameter;
		/* TODO
		 * 加载什么文件？
		 */
		parameter.loadConfigurationFile("global_planner.xml");
		/*
		 * 获取 use_quadratic 参数值，根据参数值创建 p_calc_ 实例，用 QuadraticCalculator 还是 PotentialCalculator
		 * PotentialCalculator、QuadraticCalculator
		 */
			p_calc_ = new QuadraticCalculator(cx, cy);
		/*
		 * 获取 use_dijkstra 参数值，根据参数值创建 planner_ 实例，决定用 dijkstra 算法还是 A* 算法
		 * Expander、Dijkstra、A*
		 */
		if (parameter.getParameter("use_dijkstra", 1) == 1) {
			DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
			de->setPreciseStart(true);
			planner_ = de;
		}
		/*
		 * 获取 use_grid_path 参数值，根据参数值创建 path_maker_ 实例，用 GridPath 还是 GradientPath
		 * Traceback、GridPath、GradientPath
		 */
		path_maker_ = new GradientPath(p_calc_);

		orientation_filter_ = new OrientationFilter();

//		由于 getParameter 方法没有提供返回值为 bool 类型的，因此用返回值为 int 的代替
		if (parameter.getParameter("allow_unknown", 1) == 1) {
			allow_unknown_ = true;
		} else {
			allow_unknown_ = false;
		}

		planner_->setHasUnknown(allow_unknown_); // 该方法接收一个 bool 类型参数，所有非零值都作为 true

		planner_window_x_ = parameter.getParameter("planner_window_x", 0.0f); // float 0.0f 指明调用参数为 float
		planner_window_y_ = parameter.getParameter("planner_window_y", 0.0f);
		default_tolerance_ = parameter.getParameter("default_tolerance", 0.0f);

		int lethal_cost = parameter.getParameter("lethal_cost", 253);
		int neutral_cost = parameter.getParameter("neutral_cost", 66);
		double cost_factor = parameter.getParameter("cost_factor", 0.55f);
		int orientation_mode = parameter.getParameter("orientation_mode", 1);

		planner_->setLethalCost(lethal_cost);
		path_maker_->setLethalCost(lethal_cost);
		planner_->setNeutralCost(neutral_cost);
		planner_->setFactor(cost_factor);
		orientation_filter_->setMode(orientation_mode);

		initialized_ = true;
	} else {
		printf("onInitialize has been called before\n");
	}
}

bool GlobalPlanner::makePlan(const Pose2D& start,
		const Pose2D& goal,
		std::vector<Pose2D>& plan) {
	logInfo<< "global planner start make plan";
	boost::mutex::scoped_lock lock(mutex_);

	if (!initialized_) {
		printf(
				"This planner has not been initialized yet, but it is being used, please call initialize() before use\n");
		return false;
	}
	// 先把 plan 清空
	plan.clear();

	double wx = start.x();
	double wy = start.y();

	logInfo << "start world wx = " << wx << " wy = " << wy;
	/*
	 * 把现实位置映射到地图中
	 * 获取 start 和 goal 在地图中的坐标
	 */
	unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
	double start_x, start_y, goal_x, goal_y;

	if (!costmap->getLayeredCostmap()->getCostmap()->worldToMap(wx, wy,
					start_x_i, start_y_i)) {
		// 加一下错误提示
		printf(
				"The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?\n");
		return false;
	}
	worldToMap(wx, wy, start_x, start_y);

	logInfo << "start_x_i = "<< start_x_i<<" start_y_i = "  << start_y_i;

	/*
	 * 处理 goal
	 */
	wx = goal.x();
	wy = goal.y();

	logInfo << "goal world wx = " << wx << " wy = " << wy << "\n";

	if (!costmap->getLayeredCostmap()->getCostmap()->worldToMap(wx, wy,
					goal_x_i, goal_y_i)) {
		// 加一下错误提示
		printf(
				"The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.\n");
		return false;
	}
	worldToMap(wx, wy, goal_x, goal_y);

	logInfo << "goal_x_i = "<< goal_x_i<<" goal_y_i = "<< goal_y_i;

	///clear current pose of robot at the beginning
	clearRobotCell(start_x_i, start_y_i);

	int nx = costmap->getLayeredCostmap()->getCostmap()->getSizeInCellsX(), ny =
	costmap->getLayeredCostmap()->getCostmap()->getSizeInCellsY();
	double resolution =
	costmap->getLayeredCostmap()->getCostmap()->getResolution();
	double inscribe_radius = costmap->getLayeredCostmap()->getInscribedRadius();
	double circumscribed_radius =
	costmap->getLayeredCostmap()->getCircumscribedRadius();

	logInfo << "size in cell x "<< nx<<" , ny = "<<ny << " , resolution = " << resolution
	<< " , inscribed_radius = " << inscribe_radius
	<< " ,circumscribed_radius = " << circumscribed_radius << "\n";
	logInfo << "origin x = "<< costmap->getLayeredCostmap()->getCostmap()->getOriginX()<<", y = "
	<< costmap->getLayeredCostmap()->getCostmap()->getOriginY();
	//make sure to resize the underlying array that Navfn uses
	p_calc_->setSize(nx, ny);// PotentialCalculator* p_calc_;
	planner_->setSize(nx, ny);// Expander* planner_;
	path_maker_->setSize(nx, ny);// Traceback* path_maker_;
	potential_array_ = new float[nx * ny];// float* potential_array_;

	unsigned char* char_map =
	costmap->getLayeredCostmap()->getCostmap()->getCharMap();


	///update the boundary of costmap
	outlineMap(costmap->getLayeredCostmap()->getCostmap()->getCharMap(), nx, ny,
			NS_CostMap::LETHAL_OBSTACLE);
	/*
	 * 此处开始调用算法
	 */
	bool found_legal = planner_->calculatePotentials(
			costmap->getLayeredCostmap()->getCostmap()->getCharMap(), start_x,
			start_y, goal_x, goal_y, nx * ny * 2, potential_array_);

	///计算终点周围方圆2个像素的点的potential值，防止值为POT_HIGH
	planner_->clearEndpoint(
			costmap->getLayeredCostmap()->getCostmap()->getCharMap(),
			potential_array_, goal_x_i, goal_y_i, 2);


	if (found_legal) {
		//extract the plan
		if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal,
						plan)) {
			//make sure the goal we push on has the same timestamp as the rest of the plan
			//geometry_msgs::PoseStamped goal_copy = goal;

			Pose2D goal_copy = goal;

//			goal_copy.header.stamp = NS_NaviCommon::Time::now();
			plan.push_back(goal_copy);
		} else {
			// 错误提示
			printf(
					"Failed to get a plan from potential when a legal potential was found. This shouldn't happen.\n");
		}
	} else {
		// 错误提示
		printf("Failed to get a plan.\n");
	}

	// add orientations if needed
	orientation_filter_->processPath(start, plan);
	FILE * file;
	file = fopen("/tmp/plan.log", "w");
	if (!plan.empty()) {
		for (size_t i = 0; i < plan.size(); i++) {
//        console.debug("[%d] x = %lf, y = %lf", (i + 1), plan[i].pose.position.x,
//                      plan[i].pose.position.y);
			printf("%lf,%lf,\n", plan[i].x(),
					plan[i].y());
			double map_x, map_y;
			worldToMap(plan[i].x(), plan[i].y(), map_x,
					map_y);
			fprintf(file, "%lf %lf\n", map_x, map_y);
		}
	}
	fclose(file);
	delete potential_array_;
	return !plan.empty(); // plan 非空即制订了 plan，返回 true
}

void GlobalPlanner::clearRobotCell(unsigned int mx, unsigned int my) {
	if (!initialized_) {
		// 错误提示
		printf(
				"This planner has not been initialized yet, but it is being used, please call initialize() before use\n");
		return;
	}

	//set the associated costs in the cost map to be free
	costmap->getLayeredCostmap()->getCostmap()->setCost(mx, my,
			NS_CostMap::FREE_SPACE);
}

bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y,
		double goal_x, double goal_y, const Pose2D& goal,
		std::vector<Pose2D>& plan) {
	if (!initialized_) {
		// 错误提示
		printf(
				"This planner has not been initialized yet, but it is being used, please call initialize() before use\n");
		return false;
	}
	//clear the plan, just in case
	plan.clear();

	std::vector<std::pair<float, float> > path;

	if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x,
			goal_y, path)) {
		// 错误提示
		printf("NO PATH!\n");
		return false;
	}

	NS_NaviCommon::Time plan_time = NS_NaviCommon::Time::now();
	for (int i = path.size() - 1; i >= 0; i--) {
		std::pair<float, float> point = path[i];
		//convert the plan to world coordinates
		double world_x, world_y;
		mapToWorld(point.first, point.second, world_x, world_y);

		Pose2D pose;

//		pose.header.stamp = plan_time;
//        pose.header.frame_id = global_frame;
//		pose.pose.position.x = world_x;
//		pose.pose.position.y = world_y;
//		pose.pose.position.z = 0.0;
//		pose.pose.orientation.x = 0.0;
//		pose.pose.orientation.y = 0.0;
//		pose.pose.orientation.z = 0.0;
//		pose.pose.orientation.w = 1.0;
		pose.x() = world_x;
		pose.y() = world_y;
		pose.theta() = 0;
		plan.push_back(pose);
	}

	return !plan.empty();
}

void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
	wx =
			costmap->getLayeredCostmap()->getCostmap()->getOriginX()
					+ (mx + convert_offset_)
							* costmap->getLayeredCostmap()->getCostmap()->getResolution();
	wy =
			costmap->getLayeredCostmap()->getCostmap()->getOriginY()
					+ (my + convert_offset_)
							* costmap->getLayeredCostmap()->getCostmap()->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
	double origin_x = costmap->getLayeredCostmap()->getCostmap()->getOriginX(),
			origin_y = costmap->getLayeredCostmap()->getCostmap()->getOriginY();
	double resolution =
			costmap->getLayeredCostmap()->getCostmap()->getResolution();

	if (wx < origin_x || wy < origin_y)
		return false;

	mx = (wx - origin_x) / resolution - convert_offset_;
	my = (wy - origin_y) / resolution - convert_offset_;

	if (mx < costmap->getLayeredCostmap()->getCostmap()->getSizeInCellsX()
			&& my
					< costmap->getLayeredCostmap()->getCostmap()->getSizeInCellsY())
		return true;

	return false;
}

} /* namespace NS_Planner */
