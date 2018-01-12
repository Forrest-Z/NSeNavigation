/*
 * NavigationApplication.h
 *
 *  Created on: Jan 10, 2018
 *      Author: pengjiawei
 */

#ifndef NAVIGATIONAPPLICATION_H_
#define NAVIGATIONAPPLICATION_H_

namespace NS_Navigation {
///
enum NaviState
  {
    PLANNING,
    CONTROLLING,
    CLEARING,
  };

class NavigationApplication {
public:
	NavigationApplication();
	virtual ~NavigationApplication();
};

} /* namespace NS_Navigation */

#endif /* NAVIGATIONAPPLICATION_H_ */
