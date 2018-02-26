/*
 * NavigationApplication.cpp
 *
 *  Created on: Jan 10, 2018
 *      Author: pengjiawei
 */

#include "NavigationApplication.h"
#include <Parameter/Parameter.h>
namespace NS_Navigation {

NavigationApplication::NavigationApplication() {
	// TODO Auto-generated constructor stub

}

NavigationApplication::~NavigationApplication() {
	// TODO Auto-generated destructor stub
}
void NavigationApplication::loadParameters() {
	NS_NaviCommon::Parameter parameter;

	parameter.loadConfigurationFile("navigation.xml");
	global_planner_type_ = parameter.getParameter("global_planner_type",
			"global_planner");
	local_planner_type_ = parameter.getParameter("local_planner_type",
			"trajectory_local_planner");
	planner_frequency_ = parameter.getParameter("planner_frequency", 0.0f);
	controller_frequency_ = parameter.getParameter("controller_frequency",
			5.0f);
}

bool NavigationApplication::makePlan(const NS_DataType::PoseStamped& goal,
             std::vector< NS_DataType::PoseStamped >& plan){

}
} /* namespace NS_Navigation */
