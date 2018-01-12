/*
 * NavigationApplication.h
 *
 *  Created on: Jan 10, 2018
 *      Author: pengjiawei
 */

#ifndef NAVIGATIONAPPLICATION_H_
#define NAVIGATIONAPPLICATION_H_
#include <Application/Application.h>

namespace NS_Navigation {
///
enum NaviState {
	PLANNING, CONTROLLING, CLEARING,
};
/**
 *关于导航功能的类
 */
class NavigationApplication: public Application {
public:
	NavigationApplication();
	virtual ~NavigationApplication();
private:
	/**
	 * 从文件加载参数
	 */
	void loadParameters();

private:
	std::string global_planner_type_;
	std::string local_planner_type_;
	/// 全局规划频率
	double planner_frequency_;
	/// 控制频率
	double controller_frequency_;
	/// 小车内切半径
	double inscribed_radius_;
	/// 外切半径
	double circumscribed_radius_;
};

} /* namespace NS_Navigation */

#endif /* NAVIGATIONAPPLICATION_H_ */
