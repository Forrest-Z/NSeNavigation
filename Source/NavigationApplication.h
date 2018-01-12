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
enum NaviState
  {
    PLANNING,
    CONTROLLING,
    CLEARING,
  };
/**
 *关于导航功能的类
 */
class NavigationApplication : public Application{
public:
	NavigationApplication();
	virtual ~NavigationApplication();
};

} /* namespace NS_Navigation */

#endif /* NAVIGATIONAPPLICATION_H_ */
