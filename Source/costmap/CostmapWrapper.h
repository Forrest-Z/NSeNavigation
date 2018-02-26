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
#include <vector>
namespace NS_CostMap {

/*
 *	costmap的封装类
 */
class CostmapWrapper {
public:
	CostmapWrapper();
	virtual ~CostmapWrapper();
	int a;
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
	NS_DataType::OccupancyGrid map;
	double saved_origin_x, saved_origin_y;
	///将costmap的0~255的值转为占据栅格图的-1~100
	char* cost_translation_table;
};

} /* namespace NS_CostMap */

#endif /* COSTMAP_COSTMAPWRAPPER_H_ */
