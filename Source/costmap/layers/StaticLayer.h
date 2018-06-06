#ifndef _COSTMAP_STATIC_LAYER_H_
#define _COSTMAP_STATIC_LAYER_H_

#include "../costmap_2d/CostMapLayer.h"
#include "../costmap_2d/LayeredCostMap.h"

#include <boost/thread/thread.hpp>
#include <type/map2d.h>
#include <Service/Client.h>

#include "log_tool.h"
namespace NS_CostMap {

class StaticLayer: public CostmapLayer {
public:
	StaticLayer();
	virtual
	~StaticLayer();
	virtual void
	onInitialize();
	virtual void
	activate();
	virtual void
	deactivate();
	virtual void
	reset();

	virtual void
	updateBounds(float robot_x, float robot_y, float robot_yaw,
			float* min_x, float* min_y, float* max_x, float* max_y);
	virtual void
	updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i,
			int max_j);

	virtual void
	matchSize();

private:

	unsigned char
	interpretValue(const sgbot::Map2D& new_map,int i ,int j);

private:
	unsigned int x_, y_, width_, height_;
	bool track_unknown_space_;
	bool use_maximum_;
	bool first_map_only_; ///< @brief Store the first static map and reuse it on reinitializing
	bool trinary_costmap_;

	unsigned char lethal_threshold_, unknown_cost_value_;

	float map_update_frequency_;

private:

	bool active;

	boost::thread static_layer_loop;

	bool map_received;

	bool has_updated_data;

	bool simulated;

	NS_Service::Client<sgbot::Map2D>* map_cli;

	void
	loopStaticMap();

	/**
	 * important function,update cost of static layer from laser scan
	 */
	void
	processMap(const sgbot::Map2D& new_map);

	/**
	 * if you don't have a laser,use a local pgm file for updating cost
	 */
	void readPgm(std::string pgm_file_path, int& width,
			int& height, sgbot::Map2D& value_vec);
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_STATIC_LAYER_H_
