#ifndef ODOMETRY_HELPER_ROS2_H_
#define ODOMETRY_HELPER_ROS2_H_

#include <boost/thread.hpp>
#include <DataSet/DataType/Odometry.h>
#include <DataSet/DataType/PoseStamped.h>
#include <DataSet/DataType/Twist.h>
#include <Transform/DataTypes.h>
#include <Service/ServiceType/ServiceOdometry.h>
#include <Service/Client.h>

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
    getOdom(NS_DataType::Odometry& base_odom);

    void
    getRobotVel(NS_Transform::Stamped< NS_Transform::Pose >& robot_vel);

  private:
    NS_DataType::Odometry base_odom_;
    NS_Service::Client< NS_ServiceType::ServiceOdometry >* odom_cli;

  };

} /* namespace base_local_planner */
#endif /* ODOMETRY_HELPER_ROS2_H_ */
