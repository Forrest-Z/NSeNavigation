/*
 * CostmapWrapper.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: pengjiawei
 */

#include "CostmapWrapper.h"
#include "layers/StaticLayer.h"
#include "layers/InflationLayer.h"
#include "layers/VisitedLayer.h"
#include <Time/Rate.h>
#include "utils/Footprint.h"
#include "socket_tool.h"
namespace NS_CostMap {

CostmapWrapper::CostmapWrapper() {
	// TODO Auto-generated constructor stub
	pose_cli = new NS_Service::Client<sgbot::Pose2D>("POSE");
}

CostmapWrapper::~CostmapWrapper() {
	// TODO Auto-generated destructor stub
	delete pose_cli;
}

void CostmapWrapper::updateMap() {
	// get global pose
	Pose2D pose;
	double x = 0.0, y = 0.0, yaw = 0.0;
	if (getRobotPose(pose)) {
		x = pose.x(), y = pose.y(), yaw = pose.theta();

		layered_costmap->updateMap(x, y, yaw);
	} else {
		logWarn<< "failed to get robot pose so update map with 0,0,0";
		layered_costmap->updateMap();
	}
	std::vector<Point2D> footprint;

	//by pengjiawei
	//transform the footprint to frame of robot
	transformFootprint(x, y, yaw, padded_footprint, footprint);

	footprint_for_trajectory = footprint;
//      logInfo << "footprint for trajector size = "<<footprint_for_trajectory.size();
//      setPaddedRobotFootprint (toPointVector (footprint.polygon));
//      setPaddedRobotFootprint (padded_footprint);
	//I don't want publish oriented_footprint so publish footprint directly no transform
	setPaddedRobotFootprint(footprint_from_param);

}
void CostmapWrapper::updateCostmap() {
//	prepareMap();
}
void CostmapWrapper::updateMapLoop(double frequency) {
	NS_NaviCommon::Rate rate(frequency);
	while (running) {
		updateMap();
		if (layered_costmap->isInitialized()) {
			unsigned int _x0_, _y0_, _xn_, _yn_;
			layered_costmap->getBounds(&_x0_, &_xn_, &_y0_, &_yn_);
			updateBounds(_x0_, _xn_, _y0_, _yn_);
			///useless
			updateCostmap();
		}
		rate.sleep();
		if (is_visualised) {
			visualizeForRviz();
		}
	}
}

void CostmapWrapper::visualizeForRviz() {
	int width = getCostmap()->getSizeInCellsX(), height =
			getCostmap()->getSizeInCellsY();
	double resolution = getCostmap()->getResolution();
//	logInfo << "visualize for rviz width = "<<width <<" height = "<<height<<" resolution = "<<resolution;
	std::string file_name = "/tmp/write_archive.txt";
	std::ofstream ofile(file_name, fstream::out);
	boost::archive::binary_oarchive oa(ofile);
	oa << width << height << resolution;
	unsigned char* char_map = getCostmap()->getCharMap();
	FILE* file;
	file = fopen("/tmp/rviz_costmap.log", "w");
	int index = 0;
	for (int i = 0; i < height; ++i) {
		for (int j = 0; j < width; ++j) {
			oa << char_map[index];
			fprintf(file, "%d\n", char_map[index]);
			++index;
		}
	}
	fclose(file);
	ofile.close();
	SocketSend("192.168.1.58", "12345", file_name);
}
void CostmapWrapper::loadParameters() {
	NS_NaviCommon::Parameter parameter;
	parameter.loadConfigurationFile("costmap.xml");

	if (parameter.getParameter("track_unknown_space", 1) == 1)
		track_unknown_space_ = true;
	else
		track_unknown_space_ = false;

	//is_visualised
	if (parameter.getParameter("is_visualised", 0) == 1)
		is_visualised = true;
	else
		is_visualised = false;

	footprint_ = parameter.getParameter("footprint",
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


bool CostmapWrapper::getRobotPose(Pose2D& global_pose) const {
	sgbot::Pose2D pose2d;
	pose_cli->call(pose2d);
	logInfo<< "get robot pose = ( "<< pose2d.x()<<" , "<<pose2d.y()<<" ) theta = "<<pose2d.theta();

	global_pose = pose2d;
	return true;
}

void CostmapWrapper::setPaddedRobotFootprint(
		const std::vector<Point2D>& points) {
	padded_footprint = points;
	padFootprint(padded_footprint, footprint_padding_);

	layered_costmap->setFootprint(padded_footprint);
}

void CostmapWrapper::initialize() {
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

	if(layered_costmap)
	{
		VisitedLayer* visited_layer = new VisitedLayer();
		boost::shared_ptr < CostmapLayer > layer(visited_layer);
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
	logInfo << "initial footprint.size() = "<<footprint_from_param.size();
	setPaddedRobotFootprint (footprint_from_param);

	layered_costmap->resizeMap((unsigned int)(map_width_meters_ / resolution_),
			(unsigned int)(map_height_meters_ / resolution_),
			resolution_, origin_x_, origin_y_);

}

void CostmapWrapper::start() {
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

void CostmapWrapper::stop() {
	printf("costmap is quitting!\n");

	std::vector<boost::shared_ptr<CostmapLayer> > *layers =
			layered_costmap->getPlugins();
	for (std::vector<boost::shared_ptr<CostmapLayer> >::iterator layer =
			layers->begin(); layer != layers->end(); ++layer) {
		(*layer)->deactivate();
	}

	running = false;
	update_map_thread.join();
}

} /* namespace NS_CostMap */
