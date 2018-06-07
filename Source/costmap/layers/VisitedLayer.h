/*
 * VisitedLayer.h
 *
 *  Created on: May 25, 2018
 *      Author: pengjiawei
 */

#ifndef COSTMAP_LAYERS_VISITEDLAYER_H_
#define COSTMAP_LAYERS_VISITEDLAYER_H_
#include "../costmap_2d/CostMapLayer.h"
#include "../costmap_2d/LayeredCostMap.h"

#include <Time/Rate.h>
#include <Service/Client.h>
#include <log_tool.h>
namespace NS_CostMap {
class Rect {
public:
	Rect(){

	}
	Rect(sgbot::Pose2D center_, float width_, float height_,float near_corner_tolerance_) :
			center(center_), width(width_), height(height_) {
		left_down_p = sgbot::Point2D(center.x() - width / 2,
				center.y() - height / 2);
		right_down_p = sgbot::Point2D(center.x() + width / 2,
				center.y() - height / 2);
		right_up_p = sgbot::Point2D(center.x() + width / 2,
				center.y() + height / 2);
		left_up_p = sgbot::Point2D(center.x() - width / 2,
				center.y() + height / 2);
		near_corner_tolerance = near_corner_tolerance_;
	}
	bool isInside(sgbot::Pose2D pose) {
		if (pose.x() > left_down_p.x() && pose.x() < right_down_p.x()
				&& pose.y() > left_down_p.y() && pose.y() < left_up_p.y()) {
			return true;
		} else {
			return false;
		}
	}
	//1,2,3,4
	int isNearCorner(sgbot::Pose2D pose){
		sgbot::Point2D point = sgbot::Point2D(pose.x(),pose.y());
		if( sgbot::distance(left_down_p,point) <= near_corner_tolerance){
			return 1;
		}else if( sgbot::distance(right_down_p,point) <= near_corner_tolerance ){
			return 2;
		}else if( sgbot::distance(right_up_p,point) <= near_corner_tolerance ){
			return 3;
		}else if( sgbot::distance(left_up_p,point) <= near_corner_tolerance ){
			return 4;
		}
		return 0;
	}
private:
	sgbot::Pose2D center;
	float width, height,near_corner_tolerance;
	sgbot::Point2D left_down_p, right_down_p, right_up_p, left_up_p;
};
/*
 *
 */
class VisitedLayer: public CostmapLayer {
public:
	VisitedLayer();
	virtual ~VisitedLayer();

	virtual void onInitialize();

	void activate();
	void deactivate();
	//make layer bounds to master costmap
	virtual void
	updateBounds(double robot_x, double robot_y, double robot_yaw,
			double* min_x, double* min_y, double* max_x, double* max_y);

	//update master costmap with cost of this layer
	virtual void
	updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i,
			int max_j);

	///make master costmap's size to layer
	virtual void
	matchSize();

	void loadParameters();
	void coverage();

	void markCell(const unsigned int& x, const unsigned int& y) {
		int index = x + y * size_x_;
		map_vec[index] = 1;
	}
	int searchWallPoint();
	///search wall on the front of robot
	int searchFrontWall();

	bool is_frontier_point(sgbot::Map2D& map, int index, bool* frontier_flag) {
		if (frontier_flag[index] == true) {
			return false;
		}
		int x = index % size_x_;
		int y = index / size_x_;
		//lethal obstacle and  one of 4 nbor is inscribed obstacle
		if (map.isEdge(x,y)) {
			for (int nbr : neighborhood4(index)) {
				//    if (map_[nbr] == FREE_SPACE) {
				if (map.isKnown(nbr % size_x_,nbr / size_x_)) {
					return true;
				}
			}
		}
		return false;
	}
	std::vector<int> build_frontier(sgbot::Map2D &map, int robot_index,
			int start_index, bool *frontier_flag, int limit_distance) {
		std::vector<int> frontiers;
		std::queue<int> queue1;
		queue1.push(start_index);
		while (!queue1.empty()) {
			int front = queue1.front();
			queue1.pop();
			std::vector<int> n8_vec = neighborhood4(front);
			for (int i = 0; i < n8_vec.size(); ++i) {
				if (is_frontier_point(map, n8_vec[i], frontier_flag)) {
					if (distance(robot_index % size_x_, robot_index / size_x_,
							n8_vec[i] % size_x_, n8_vec[i] / size_x_))
						frontier_flag[n8_vec[i]] = true;
					frontiers.push_back(n8_vec[i]);
					queue1.push(n8_vec[i]);
				}
			}
		}
		FILE* file = fopen("../frontier.log", "a+");
		for (auto iterator = frontiers.begin(); iterator != frontiers.end();
				++iterator) {
			fprintf(file, "%d %d\n", *iterator % size_x_, *iterator / size_x_);
		}
		fclose(file);
		printf("frontiers size = %d\n", frontiers.size());
		return frontiers;
	}
	std::vector<int> neighborhood4(int idx) {
		std::vector<int> out;

		if (idx > size_x_ * size_y_ - 1) {
			printf("Evaluating nhood for offmap point\n");
			return out;
		}

		if (idx % size_x_ > 0) {
			out.push_back(idx - 1);
		}
		if (idx % size_x_ < size_x_ - 1) {
			out.push_back(idx + 1);
		}
		if (idx >= size_x_) {
			out.push_back(idx - size_x_);
		}
		if (idx < size_x_ * (size_y_ - 1)) {
			out.push_back(idx + size_x_);
		}
		return out;
	}
	//6 4 8
	//1 * 2
	//5 3 7
	std::vector<int> neighborhood8(int idx) {
		std::vector<int> out = neighborhood4(idx);

		if (idx > size_x_ * size_y_ - 1) {
			return out;
		}

		if (idx % size_x_ > 0 && idx >= size_x_) {
			out.push_back(idx - 1 - size_x_);
		}
		if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
			out.push_back(idx - 1 + size_x_);
		}
		if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
			out.push_back(idx + 1 - size_x_);
		}
		if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
			out.push_back(idx + 1 + size_x_);
		}

		return out;
	}
	int distance(int x1, int y1, int x2, int y2) {
		int d = static_cast<int>(std::sqrt(
				std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2)));
		return d;
	}
	Rect* generateRectangle(const sgbot::Pose2D& center) {
		rect_p = new Rect(center,sweep_rec_x,sweep_rec_y,near_corner_tolerance);
		return rect_p;
	}
private:
	bool active;
	std::vector<int> map_vec;
	std::vector<int> index_vec;
	NS_Service::Client<sgbot::Pose2D>* pose_cli;
	NS_Service::Client<sgbot::Map2D>* map_cli;
	boost::thread coverage_loop;
	int times = 0;
	//unit m
	float sweep_rec_x, sweep_rec_y;
	//frontier threshold
	int frontiers_threshold;
	//current rectangle
	Rect* rect_p;
	//near corner tolerance
	float near_corner_tolerance;
};

} /* namespace NS_CostMap */

#endif /* COSTMAP_LAYERS_VISITEDLAYER_H_ */
