#ifndef _COSTMAP_COSTMAP_LAYER_H_
#define _COSTMAP_COSTMAP_LAYER_H_

#include "CostMap2D.h"
#include "LayeredCostMap.h"

namespace NS_CostMap {

class LayeredCostmap;
/**
 * costmap layer 的封装类
 */
class CostmapLayer: public Costmap2D {
public:
	CostmapLayer();

	void
	initialize(LayeredCostmap* parent);

	/**
	 * 更新地图边界
	 */
	virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
			double* min_x, double* min_y, double* max_x, double* max_y) {
	}

	/**
	 */
	virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j,
			int max_i, int max_j) {
	}

	/// 禁用某一层
	virtual void deactivate() {
	}

	///启用某一层
	virtual void activate() {
	}

	virtual void reset() {
	}


	/**
	 */
	bool isCurrent() const {
		return current_;
	}

	/** @brief for child layer matching their size for master costmap*/
	virtual void matchSize() {
	}

	/** @brief just for layered_costmap_->getFootprint(). */
	const std::vector<sgbot::sensor::Point2D>&
	getFootprint() const;

	/** @brief called when the foorprint is changed , called in LayeredCostmap::setFootprint())  */
	virtual void onFootprintChanged() {
	}

protected:
	   /** @brief method for child layer initialize */
	    virtual void onInitialize()
	    {
	    }
	/*
	 * 用这一层的真实值写入到主costmap中
	 */
	void
	updateWithTrueOverwrite(Costmap2D& master_grid, int min_i, int min_j,
			int max_i, int max_j);

	LayeredCostmap* layered_costmap_;
	bool current_;
	bool enabled_;

	bool has_extra_bounds_;
	///actually we have never used the extra bound so we don't need
	///extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
private:
	std::vector<sgbot::sensor::Point2D> footprint_spec_;
};

}  // namespace costmap_2d
#endif  // COSTMAP_2D_COSTMAP_LAYER_H_
