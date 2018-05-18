#include "../layers/StaticLayer.h"

#include "../utils/Math.h"
#include <Parameter/Parameter.h>
#include <Console/Console.h>

#include <Time/Rate.h>

using NS_CostMap::NO_INFORMATION;
using NS_CostMap::LETHAL_OBSTACLE;
using NS_CostMap::FREE_SPACE;

namespace NS_CostMap {

StaticLayer::StaticLayer() {
	map_cli = new NS_Service::Client<sgbot::Map2D>("MAP");
	active = false;
}

StaticLayer::~StaticLayer() {
	delete map_cli;
}

void StaticLayer::loopStaticMap() {
	NS_NaviCommon::Rate rate(map_update_frequency_);
	while (active) {
		sgbot::Map2D srv_map;

		if (simulated) {
			std::string file_path = "/home/pengjiawei/map.pgm";
//			logInfo << "simulated so please located pgm file in path = "<<file_path;
			//resolution
			srv_map.resolution_ = 1.f;
			srv_map.origin_.x() = 0.0;
			srv_map.origin_.y() = 0.0;
			int width,height;
			readPgm(file_path, width,height,srv_map);
			srv_map.width_ = width;
			srv_map.height_ = height;
//			for(int i = 0; i < srv_map.map.data.size();++i){
//				srv_map.map.data[i] = '0';
//			}
			processMap(srv_map);
		} else {
			if (map_cli->call(srv_map)) {
					processMap(srv_map);
			}else{
				logInfo<<"static layer call map failed";
			}
		}

		rate.sleep();
	}
}

void StaticLayer::onInitialize() {

	NS_NaviCommon::Parameter parameter;

	parameter.loadConfigurationFile("static_layer.xml");

	if (parameter.getParameter("track_unknown_space", 1) == 1)
		track_unknown_space_ = true;
	else
		track_unknown_space_ = false;

	if (parameter.getParameter("use_maximum", 0) == 1)
		use_maximum_ = true;
	else
		use_maximum_ = false;

	if (parameter.getParameter("simulated",0) == 1)
		simulated = true;
	else
		simulated = false;

	int temp_lethal_threshold;
	temp_lethal_threshold = parameter.getParameter("lethal_threshold", 100);
	lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);

	unknown_cost_value_ = parameter.getParameter("unknown_cost_value", -1);

	if (parameter.getParameter("trinary_costmap", 1) == 1)
		trinary_costmap_ = true;
	else
		trinary_costmap_ = false;

	map_update_frequency_ = parameter.getParameter("map_update_frequency",
			1.0f);

	x_ = y_ = 0;
	width_ = height_ = 0;

	enabled_ = true;
}

void StaticLayer::matchSize() {
	Costmap2D* master = layered_costmap_->getCostmap();
	resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
			master->getResolution(), master->getOriginX(),
			master->getOriginY());

}

unsigned char StaticLayer::interpretValue(const sgbot::Map2D& new_map,int i,int j) {
	// check if the static value is above the unknown or lethal thresholds
	if (track_unknown_space_ && new_map.isUnknown(i,j))
		return NO_INFORMATION;
	else if (!track_unknown_space_ && new_map.isUnknown(i,j))
		return FREE_SPACE;
	else if(new_map.isEdge(i,j))
//	else if (value >= lethal_threshold_)
		return LETHAL_OBSTACLE;
	else if (trinary_costmap_)
		return FREE_SPACE;

	double scale = (double) new_map.getPoint(i,j) / lethal_threshold_;
	return scale * LETHAL_OBSTACLE;
}

void StaticLayer::processMap(const sgbot::Map2D& new_map) {
	unsigned int size_x = new_map.getWidth(), size_y = new_map.getHeight();

    logInfo<<"Received size_x = "<<size_x<<" size_y = "<<size_y << ",resolution = "<<new_map.getResolution();

// resize costmap if size, resolution or origin do not match
	Costmap2D* master = layered_costmap_->getCostmap();
	if (master->getSizeInCellsX() != size_x
			|| master->getSizeInCellsY() != size_y
			|| master->getResolution() != new_map.getResolution()
			|| master->getOriginX() != new_map.getOrigin().x()
			|| master->getOriginY() != new_map.getOrigin().y()
			|| !layered_costmap_->isSizeLocked()) {
		// Update the size of the layered costmap (and all layers, including this one)
		printf("Resizing costmap to %d X %d at %f m/pix\n", size_x, size_y, new_map.getResolution());
		layered_costmap_->resizeMap(size_x, size_y, new_map.getResolution(),
				new_map.getOrigin().x(), new_map.getOrigin().y(),
				true);
	}
	if (size_x_ != size_x || size_y_ != size_y
			|| resolution_ != new_map.getResolution()
			|| origin_x_ != new_map.getOrigin().x()
			|| origin_y_ != new_map.getOrigin().y()) {
		// only update the size of the costmap stored locally in this layer
		printf("Resizing static layer to %d X %d at %f m/pix\n", size_x, size_y, new_map.getResolution());
		resizeMap(size_x, size_y, new_map.getResolution(),
				new_map.getOrigin().x(), new_map.getOrigin().y());
	}

	unsigned int index = 0;

	FILE* file = fopen("/tmp/static_layer_costmap.log", "w+");




	// initialize the costmap with static data
	for (unsigned int i = 0; i < size_y; ++i) {
		for (unsigned int j = 0; j < size_x; ++j) {
			fprintf(file, "%d\n", interpretValue(new_map,j,i));
				costmap_[index] = interpretValue(new_map,j,i);
			++index;
		}
	}
	fclose(file);
	// we have a new map, update full size of map
	x_ = y_ = 0;
	width_ = size_x_;
	height_ = size_y_;
	map_received = true;
	has_updated_data = true;
}

void StaticLayer::activate() {
	active = true;
	static_layer_loop = boost::thread(
			boost::bind(&StaticLayer::loopStaticMap, this));
}

void StaticLayer::deactivate() {
	active = false;
	static_layer_loop.join();
}

void StaticLayer::reset() {
	deactivate();
	activate();
}

void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
		double* min_x, double* min_y, double* max_x, double* max_y) {
	if (!map_received || !(has_updated_data || has_extra_bounds_)) {
//		printf("Not update bounds.\n");
		return;
	}

//    useExtraBounds(min_x, min_y, max_x, max_y);

	double wx, wy;

	mapToWorld(x_, y_, wx, wy);
	*min_x = std::min(wx, *min_x);
	*min_y = std::min(wy, *min_y);

	mapToWorld(x_ + width_, y_ + height_, wx, wy);
	*max_x = std::max(wx, *max_x);
	*max_y = std::max(wy, *max_y);

	has_updated_data = false;
}

void StaticLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j,
		int max_i, int max_j) {
	if (!map_received) {
		printf("Not update costs.\n");
		return;
	}

	// if not rolling, the layered costmap (master_grid) has same coordinates as this layer
	if (!use_maximum_)
		updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
//    else
//      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}
void StaticLayer::readPgm(std::string pgm_file_path, int& width,
		int& height, sgbot::Map2D& map) {
	int row = 0, col = 0;
	ifstream infile(pgm_file_path);
	stringstream ss;
	string inputLine = "";

	// First line : version
	getline(infile, inputLine);
	if (inputLine.compare("P5") != 0)
		logError << "Version error" << endl;
	else
//		cout << "Version : " << inputLine << endl;

	// Second line : comment
	getline(infile, inputLine);

	while (inputLine.find("#") != string::npos) {
//		cout << "Comment : " << inputLine << endl;
		getline(infile, inputLine);
	}
	// Continue with a stringstream
	ss << infile.rdbuf();
	// Third line : size
	stringstream wwhhss(inputLine);
	wwhhss >> width >> height;
	logInfo << width << " columns and " << height << " rows";

	int array[height][width];

	map.resize(width,height);
	int max_value;
	ss >> max_value;
//	logInfo << "max value = " << max_value << endl;
	unsigned char pixel;
	int index = 0;
	// Following lines : data
	for (row = 0; row < height; ++row) {
		for (col = 0; col < width; ++col) {
			ss >> pixel;
			array[row][col] = pixel;
			///transfer value of pgm to occupancy grid
			    if(pixel == 255){
			    	map.updateAsKnown(col,row);
			    }else if(pixel == 205){
			    	pixel = -1;
			    }else if(pixel == 0){
			    	map.updateAsUnknown(col,row);
			    }
		++index;
		}
	}
	// Now print the array to see the result
//	FILE* file = fopen("/home/pengjiawei/static_layer_costmap.log", "a");
//	for (row = 0; row < height; ++row) {
//		for (col = 0; col < width; ++col) {
//			fprintf(file, "%d\n", array[row][col]);
//		}
//	}
//
//	fclose(file);
	infile.close();
}

}  // namespace costmap_2d
