/*
 * CostmapWrapper.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: pengjiawei
 */

#include "CostmapWrapper.h"
#include "layers/StaticLayer.h"
#include "layers/InflationLayer.h"

#include <DataSet/DataType/PolygonStamped.h>
#include <Time/Rate.h>
#include "utils/Footprint.h"
namespace NS_CostMap {

CostmapWrapper::CostmapWrapper() {
	// TODO Auto-generated constructor stub

}

CostmapWrapper::~CostmapWrapper() {
	// TODO Auto-generated destructor stub
}

void CostmapWrapper::updateMap()
  {
    // get global pose
    NS_Transform::Stamped < NS_Transform::Pose > pose;
    double x = 0.0,y = 0.0,yaw = 0.0;
    if(getRobotPose (pose))
    {
      x = pose.getOrigin().x(), y = pose.getOrigin().y(),
          yaw = NS_Transform::getYaw(pose.getRotation());

      layered_costmap->updateMap(x, y, yaw);
    }else{
    	logWarn << "failed to get robot pose so update map with 0,0,0";
    	layered_costmap->updateMap();
    }
      NS_DataType::PolygonStamped footprint;
      footprint.header.stamp = NS_NaviCommon::Time::now();

      //by pengjiawei
      logInfo <<"update map padded footprint size = "<<padded_footprint.size();
      transformFootprint(x, y, yaw, padded_footprint, footprint);

      footprint_for_trajectory = toPointVector(footprint.polygon);
//      setPaddedRobotFootprint (toPointVector (footprint.polygon));
//      setPaddedRobotFootprint (padded_footprint);
      setPaddedRobotFootprint (footprint_from_param);

  }
void CostmapWrapper::updateCostmap(){

}
void CostmapWrapper::updateMapLoop(double frequency)
{
  NS_NaviCommon::Rate rate(frequency);
  while(running)
  {
    updateMap();
    if(layered_costmap->isInitialized())
    {
      unsigned int _x0_, _y0_, _xn_, _yn_;
      layered_costmap->getBounds(&_x0_, &_xn_, &_y0_, &_yn_);
      updateBounds(_x0_, _xn_, _y0_, _yn_);
      ///useless
      updateCostmap();
    }
    rate.sleep();
  }
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
       "[[0.16, 0.16], [0.16, -0.16], [-0.16, 0]]");

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


bool CostmapWrapper::getRobotPose(
      NS_Transform::Stamped< NS_Transform::Pose >& global_pose) const
  {
    NS_ServiceType::ServiceTransform odom_transform;
    NS_ServiceType::ServiceTransform map_transform;

    int times = 0;
    while(times != 3)
    {
      NS_Service::Client < NS_ServiceType::ServiceTransform > map_tf_cli(
          "ODOM_MAP_TF");
      if(map_tf_cli.call(map_transform) == true)
      {
        break;
      }
      ++times;
    }
    if(times == 3)
    {
      return false;
    }

    times = 0;
    while(times != 3)
    {
      NS_Service::Client < NS_ServiceType::ServiceTransform > odom_tf_cli(
          "BASE_ODOM_TF");
      if(odom_tf_cli.call(odom_transform) == true)
      {
        break;
      }
      ++times;
    }
    if(times == 3)
    {
      return false;
    }

    //TODO: not verify code for transform
    NS_Transform::Transform odom_tf, map_tf;
    NS_Transform::transformMsgToTF(odom_transform.transform, odom_tf);
    NS_Transform::transformMsgToTF(map_transform.transform, map_tf);

    global_pose.setData(odom_tf * map_tf);

    logInfo << "get robot pose successfully";
    return true;
  }




void CostmapWrapper::setPaddedRobotFootprint(
    const std::vector< NS_DataType::Point >& points)
{
  padded_footprint = points;
  padFootprint(padded_footprint, footprint_padding_);

  layered_costmap->setFootprint(padded_footprint);
}


void CostmapWrapper::initialize()
 {
	logInfo<<"costmap wrapper initialize";
   loadParameters();

   layered_costmap = new LayeredCostmap(track_unknown_space_);

   if(layered_costmap)
   {
     StaticLayer* static_layer = new StaticLayer();
     boost::shared_ptr < CostmapLayer > layer(static_layer);
     layered_costmap->addPlugin(layer);
   }

   if(layered_costmap)
   {
     InflationLayer* inflation_layer = new InflationLayer();
     boost::shared_ptr < CostmapLayer > layer(inflation_layer);
     layered_costmap->addPlugin(layer);
   }

   std::vector < boost::shared_ptr< CostmapLayer > > *layers = layered_costmap->getPlugins();
   for(std::vector< boost::shared_ptr< CostmapLayer > >::iterator layer = layers->begin();
       layer != layers->end(); ++layer)
   {
     (*layer)->initialize(layered_costmap);
   }

   xn = yn = 0;
   x0 = layered_costmap->getCostmap()->getSizeInCellsX();
   y0 = layered_costmap->getCostmap()->getSizeInCellsY();

    if(!makeFootprintFromString(footprint_, footprint_from_param))
    {
      printf("Footprint parameter parse failure!\n");
      return;
    }

//   footprint_from_param = makeFootprintFromRadius(footprint_radius);
   logInfo << "initial footprint.size() = "<<footprint_from_param.size();
   setPaddedRobotFootprint (footprint_from_param);


   layered_costmap->resizeMap((unsigned int)(map_width_meters_ / resolution_),
                              (unsigned int)(map_height_meters_ / resolution_),
                              resolution_, origin_x_, origin_y_);

 }

 void CostmapWrapper::start()
 {
   logInfo<<"costmap is running!";

   std::vector < boost::shared_ptr< CostmapLayer > > *layers = layered_costmap->getPlugins();
   for(std::vector< boost::shared_ptr< CostmapLayer > >::iterator layer = layers->begin();
       layer != layers->end(); ++layer)
   {
     (*layer)->activate();
   }

   running = true;

   update_map_thread = boost::thread(
       boost::bind(&CostmapWrapper::updateMapLoop, this,
                   map_update_frequency_));
 }

 void CostmapWrapper::stop()
 {
   printf("costmap is quitting!\n");

   std::vector < boost::shared_ptr< CostmapLayer > > *layers = layered_costmap->getPlugins();
   for(std::vector< boost::shared_ptr< CostmapLayer > >::iterator layer = layers->begin();
       layer != layers->end(); ++layer)
   {
     (*layer)->deactivate();
   }

   running = false;
   update_map_thread.join();
 }


} /* namespace NS_CostMap */
