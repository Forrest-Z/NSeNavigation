#ifndef ODOMETRY_HELPER_ROS2_H_
#define ODOMETRY_HELPER_ROS2_H_

#include <boost/thread.hpp>

#include <transform/transform2d.h>
#include <type/odometry.h>

#include <Service/Client.h>
using namespace sgbot;
namespace NS_Planner
{

  class OdometryHelper
  {
  public:

    /** @brief Constructor.
     * @param odom_topic The topic on which to subscribe to Odometry
     *        messages.  If the empty string is given (the default), no
     *        subscription is done. */
    OdometryHelper();
    ~OdometryHelper();

    /**
     * @brief  Callback for receiving odometry data
     * @param msg An Odometry message
     */
    void
    getOdom(Odometry& base_odom);

    void
    getRobotVel(Velocity2D& robot_vel);

  private:
    Odometry base_odom_;
    NS_Service::Client< Odometry >* odom_cli;

  };

} /* namespace base_local_planner */
#endif /* ODOMETRY_HELPER_ROS2_H_ */
