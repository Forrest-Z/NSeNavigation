#include <Transform/DataTypes.h>
#include <DataSet/DataType/Odometry.h>
#include "OdometryHelper.h"

namespace NS_Planner
{

  OdometryHelper::OdometryHelper()
  {
    odom_cli = new NS_Service::Client< NS_ServiceType::ServiceOdometry >(
        "BASE_ODOM");
  }

  OdometryHelper::~OdometryHelper()
  {
    delete odom_cli;
  }

  void OdometryHelper::getRobotVel(
      NS_Transform::Stamped< NS_Transform::Pose >& robot_vel)
  {
    // Set current velocities from odometry
    NS_DataType::Twist global_vel;

    NS_ServiceType::ServiceOdometry odom_rep;

    if(odom_cli->call(odom_rep) && odom_rep.result)
    {
      base_odom_ = odom_rep.odom;

      global_vel.linear.x = base_odom_.twist.linear.x;
      global_vel.linear.y = base_odom_.twist.linear.y;
      global_vel.angular.z = base_odom_.twist.angular.z;

      robot_vel.setData(
          NS_Transform::Transform(
              NS_Transform::createQuaternionFromYaw(global_vel.angular.z),
              NS_Transform::Vector3(global_vel.linear.x, global_vel.linear.y,
                                    0)));
      robot_vel.stamp_ = NS_NaviCommon::Time();
    }
  }

  void OdometryHelper::getOdom(NS_DataType::Odometry& base_odom)
  {
    NS_ServiceType::ServiceOdometry odom_rep;

    if(odom_cli->call(odom_rep) && odom_rep.result)
    {
      base_odom_ = odom_rep.odom;
      base_odom = base_odom_;
    }
  }

} /* namespace base_local_planner */
