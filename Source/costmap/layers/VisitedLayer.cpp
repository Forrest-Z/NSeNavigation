/*
 * VisitedLayer.cpp
 *
 *  Created on: May 25, 2018
 *      Author: pengjiawei
 */

#include <costmap/layers/VisitedLayer.h>

namespace NS_CostMap {

VisitedLayer::VisitedLayer() {
	// TODO Auto-generated constructor stub
	pose_cli = new NS_Service::Client<sgbot::Pose2D>("POSE");
	map_cli = new NS_Service::Client<sgbot::Map2D>("MAP");
	rect_p = new Rect();
}

VisitedLayer::~VisitedLayer() {
	// TODO Auto-generated destructor stub
	delete pose_cli;
	delete map_cli;
	delete rect_p;
}
void VisitedLayer::loadParameters(){
	NS_NaviCommon::Parameter parameter;

	parameter.loadConfigurationFile("visited_layer.xml");
	sweep_rec_x = parameter.getParameter("sweep_rec_x",2.f);
	sweep_rec_y = parameter.getParameter("sweep_rec_y",2.f);
	frontiers_threshold = parameter.getParameter("frontiers_threshold",10);
}

void VisitedLayer::onInitialize() {
	logInfo<< "visited layer on initialize";
	loadParameters();
}
void VisitedLayer::activate() {
	active = true;
	coverage_loop = boost::thread(boost::bind(&VisitedLayer::coverage, this));
}

void VisitedLayer::deactivate() {
	active = false;
	coverage_loop.join();
}
void VisitedLayer::matchSize() {
	NS_CostMap::Costmap2D* costmap = layered_costmap_->getCostmap();
	unsigned int size_x = costmap->getSizeInCellsX();
	unsigned int size_y = costmap->getSizeInCellsY();
	if (map_vec.size() != size_x * size_y) {
		map_vec.resize(size_x * size_y);
	}
	size_x_ = size_x;
	size_y_ = size_y;
}
void VisitedLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j,
		int max_i, int max_j) {

}
void VisitedLayer::updateBounds(double robot_x, double robot_y,
		double robot_yaw, double* min_x, double* min_y, double* max_x,
		double* max_y) {

}
void VisitedLayer::coverage() {
	sgbot::Pose2D pose;
	NS_NaviCommon::Rate rate(1.f);
	while (active) {
		if (pose_cli->call(pose)) {
			unsigned int map_x, map_y;
			layered_costmap_->getCostmap()->worldToMap(pose.x(), pose.y(),
					map_x, map_y);
			logInfo<< "pose = "<<pose.x()<<" , "<<pose.y()<<" . map_x = "<<map_x<<" , "<<map_y;
			int index = map_x + map_y * size_x_;
			if ( std::find(index_vec.begin(), index_vec.end(), index) != index_vec.end() ) {
				index_vec.push_back(index);
				logInfo<<"index vec size = "<<index_vec.size();
			}
			markCell(map_x, map_y);
		}
		rate.sleep();
	}
}
int VisitedLayer::searchWall() {
	sgbot::Pose2D pose;
	sgbot::Map2D map;
	if (!pose_cli->call(pose) || map_cli->call(map) ) {
		logInfo<<"search wall call pose failed";
	} else {
		std::queue<int> queue1;
		int robot_index = pose.x() + pose.y() * size_x_;
		bool visited[size_x_ * size_y_] = {false};
		bool frontier_flag[size_x_ * size_y_] = {false};
		queue1.push(robot_index);
		while (!queue1.empty()) {
			int front = queue1.front();
			queue1.pop();
			visited[front] = true;
			std::vector<int> neithbor_vec = neighborhood4(front);
			for (int i = 0; i < neithbor_vec.size(); ++i) {
				if (is_frontier_point(map, neithbor_vec[i], frontier_flag)) {
					//this is the first frontier point so we need record it
					int first_frontier = neithbor_vec[i];
					printf("%d,%d\n", neithbor_vec[i] % size_x_, neithbor_vec[i] / size_x_);
					frontier_flag[neithbor_vec[i]] = true;
					if (times++ == 0) {
						int d = distance(neithbor_vec[i] % size_x_, neithbor_vec[i] / size_x_,
								pose.x(), pose.y());
						std::vector<int> frontiers = build_frontier(map, robot_index, neithbor_vec[i],
								frontier_flag, d);
						if(frontiers.size() > frontiers_threshold){
							return first_frontier;
						}
					}
				}
				if (!visited[neithbor_vec[i]]) {
					visited[neithbor_vec[i]] = true;
					queue1.push(neithbor_vec[i]);
				}
			}
		}
	}

}

}
/* namespace NS_CostMap */
