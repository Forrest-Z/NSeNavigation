#include <Transform/DataTypes.h>
#include <DataSet/DataType/Odometry.h>
#include "OdometryHelper.h"

#include <transform/transform2d.h>
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
		  Velocity2D& robot_vel)
  {
    // Set current velocities from odometry
    Velocity2D global_vel;

    NS_ServiceType::ServiceOdometry odom_rep;

    if(odom_cli->call(odom_rep) && odom_rep.result)
    {
      base_odom_ = odom_rep.odom;

      global_vel.linear = base_odom_.twist.linear.x;
//      global_vel.linear.y = base_odom_.twist.linear.y;
      global_vel.angular = base_odom_.twist.angular.z;

//      robot_vel.setData(
//          NS_Transform::Transform(
//              NS_Transform::createQuaternionFromYaw(global_vel.angular.z),
//              NS_Transform::Vector3(global_vel.linear.x, global_vel.linear.y,
//                                    0)));
      robot_vel.linear = global_vel.linear;
      robot_vel.angular = global_vel.angular;

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
