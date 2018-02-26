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

	/** @brief Implement this to make this layer match the size of the parent costmap. */
	virtual void matchSize() {
	}

	/** @brief Convenience function for layered_costmap_->getFootprint(). */
	const std::vector<NS_DataType::Point>&
	getFootprint() const;

	/** @brief LayeredCostmap calls this whenever the footprint there
	 * changes (via LayeredCostmap::setFootprint()).  Override to be
	 * notified of changes to the robot's footprint. */
	virtual void onFootprintChanged() {
	}

protected:
	   /** @brief This is called at the end of initialize().  Override to
	     * implement subclass-specific initialization.
	     *
	     * tf_, name_, and layered_costmap_ will all be set already when this is called. */
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
	bool enabled_; ///< Currently this var is managed by subclasses. TODO: make this managed by this class and/or container class.
private:
	std::vector<NS_DataType::Point> footprint_spec_;
};

}  // namespace costmap_2d
#endif  // COSTMAP_2D_COSTMAP_LAYER_H_
