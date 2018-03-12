/*
 * CostmapWrapper.h
 *
 *  Created on: Jan 11, 2018
 *      Author: pengjiawei
 */

#ifndef COSTMAP_COSTMAPWRAPPER_H_
#define COSTMAP_COSTMAPWRAPPER_H_
#include "costmap_2d/CostMapLayer.h"
#include <Parameter/Parameter.h>
#include <DataSet/DataType/OccupancyGrid.h>

#include <Service/ServiceType/ServiceBase.h>
#include <Service/ServiceType/ServiceTransform.h>
#include <Service/Client.h>
#include "log_tool.h"
#include "socket_tool.h"
#include "transform/transform2d.h"
//class MapMetaData{
//public:
//	MapMetaData():  resolution(0.0), width(0), height(0), origin(){
//
//	}
//	NS_DataType::Pose origin;
//	   float resolution;
//	    int16_t width;
//	    int16_t height;
//};
//class OccupancyGrid{
//	public:
//	OccupancyGrid():info(), data(){
//
//	}
//	std::vector<char> data;
//	MapMetaData info;
//};
namespace NS_CostMap {

/*
 *	costmap的封装类
 */
class CostmapWrapper {
public:
	CostmapWrapper();
	virtual ~CostmapWrapper();

	    NS_Service::Client< NS_ServiceType::ServiceTransform >* odom_tf_cli;

	    NS_Service::Client< NS_ServiceType::ServiceTransform >* map_tf_cli;
private:
	void
	loadParameters();
	/**
	 * 扩充地图边界
	 */
	void updateBounds(unsigned int x0_, unsigned int xn_, unsigned int y0_,
			unsigned int yn_) {
		x0 = std::min(x0, x0_);
		xn = std::max(xn, xn_);
		y0 = std::min(y0, y0_);
		yn = std::max(yn, yn_);
	}
	/**
	 * 初始化占据栅格图
	 */
	void
	prepareMap();

	/**
	 * update footprint
	 */
	void
	setPaddedRobotFootprint(const std::vector<sgbot::sensor::Point2D>& points);

	/**
	 * call layered_costmap.updateMap mainly
	 */
	void
	updateMap();
	/**
	 * useless?
	 */
	void
	updateCostmap();

	void
	updateMapLoop(double frequency);

	void visualizeForRviz();
private:
	///决定costmap的默认值为未知还是空白
	bool track_unknown_space_;
	///小车的足迹
	std::string footprint_;
	///costmap的更新频率
	double map_update_frequency_;
	double map_width_meters_;
	double map_height_meters_;
	double resolution_;
	///图的原点
	double origin_x_;
	///图的原点
	double origin_y_;
	///扩充小车足迹
	float footprint_padding_;
	///圆形小车的半径
	double footprint_radius;

	unsigned int x0, xn, y0, yn;

	LayeredCostmap* layered_costmap;


	boost::thread update_map_thread;

	NS_DataType::OccupancyGrid map;
	double saved_origin_x, saved_origin_y;
	///将costmap的0~255的值转为占据栅格图的-1~100
	char* cost_translation_table;

    std::vector< sgbot::sensor::Point2D > padded_footprint;

    std::vector< sgbot::sensor::Point2D > footprint_for_trajectory;

    std::vector< sgbot::sensor::Point2D > footprint_from_param;

    bool got_map;

    bool running;
public:
	LayeredCostmap*
	getLayeredCostmap() {
		return layered_costmap;
	}
	;
	bool
	getRobotPose(sgbot::tf::Pose2D& global_pose) const;

	Costmap2D*
	getCostmap() {
		return layered_costmap->getCostmap();
	}
	;
	std::vector<sgbot::sensor::Point2D> getRobotFootprint() {
		//      return padded_footprint;
		return footprint_for_trajectory;
	}
public:
	void
	initialize();
	void
	start();
	void
	stop();
};

} /* namespace NS_CostMap */

#endif /* COSTMAP_COSTMAPWRAPPER_H_ */
