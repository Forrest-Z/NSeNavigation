/*
 * CostmapWrapper.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: pengjiawei
 */

#include "CostmapWrapper.h"

namespace NS_CostMap {

CostmapWrapper::CostmapWrapper() {
	// TODO Auto-generated constructor stub

}

CostmapWrapper::~CostmapWrapper() {
	// TODO Auto-generated destructor stub
}
void CostmapWrapper::loadParameters()
 {
   NS_NaviCommon::Parameter parameter;
   parameter.loadConfigurationFile("costmap.xml");

   if(parameter.getParameter("track_unknown_space", 1) == 1)
     track_unknown_space_ = true;
   else
     track_unknown_space_ = false;

   footprint_ = parameter.getParameter(
       "footprint",
       "[[0.16, 0.16], [0.16, -0.16], [-0.16, -0.16], [-0.16, 0.16]]");

   footprint_radius = parameter.getParameter("footprint_radius", 0.16f);

   map_width_meters_ = parameter.getParameter("map_width", 6.0f);
   map_height_meters_ = parameter.getParameter("map_height", 6.0f);
   resolution_ = parameter.getParameter("resolution", 0.01f);

   map_update_frequency_ = parameter.getParameter("map_update_frequency",
                                                  1.0f);
   footprint_padding_ = parameter.getParameter("footprint_padding_", 0.1f);
   origin_x_ = 0.0;
   origin_y_ = 0.0;
 }

void CostmapWrapper::prepareMap()
  {
    Costmap2D* costmap_ = layered_costmap->getCostmap();

    boost::unique_lock < Costmap2D::mutex_t > lock(*(costmap_->getMutex()));

    double resolution = costmap_->getResolution();

    map.header.stamp = NS_NaviCommon::Time::now();
    map.info.resolution = resolution;

    map.info.width = costmap_->getSizeInCellsX();
    map.info.height = costmap_->getSizeInCellsY();

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    map.info.origin.position.x = wx - resolution / 2;
    map.info.origin.position.y = wy - resolution / 2;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    saved_origin_x = costmap_->getOriginX();
    saved_origin_y = costmap_->getOriginY();

    map.data.resize(map.info.width * map.info.height);

    unsigned char* data = costmap_->getCharMap();
    for(unsigned int i = 0; i < map.data.size(); i++)
    {
      map.data[i] = cost_translation_table[data[i]];
    }
  }

} /* namespace NS_CostMap */
