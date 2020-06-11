/**************************
 *      WOLF includes     *
 **************************/
#include <core/capture/capture_odom_2d.h>
#include <core/sensor/sensor_odom_2d.h>
#include <core/processor/processor_odom_2d.h>

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <iomanip>
#include <queue>

#include "subscriber.h"
#include "subscriber_odom2d.h"

namespace wolf
{
SubscriberOdom2d::SubscriberOdom2d(const std::string& _unique_name,
                                   const ParamsServer& _server,
                                   const SensorBasePtr _sensor_ptr)
  : Subscriber(_unique_name, _server, _sensor_ptr)
  , last_odom_stamp_(ros::Time(0))
  , odometry_translational_cov_factor_(std::static_pointer_cast<SensorOdom2d>(_sensor_ptr)->getDispVarToDispNoiseFactor())
  , odometry_rotational_cov_factor_(std::static_pointer_cast<SensorOdom2d>(_sensor_ptr)->getRotVarToRotNoiseFactor())
{
}

void SubscriberOdom2d::initSubscriber(ros::NodeHandle& nh, const std::string& topic)
{
    sub_ = nh.subscribe(topic, 100, &SubscriberOdom2d::callback, this);
}

void SubscriberOdom2d::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_DEBUG("WolfNodePolyline::odomCallback");
    ROS_INFO("WolfNodePolyline::odomCallback: start");

    if (last_odom_stamp_ != ros::Time(0))
    {
        double           dt          = (msg->header.stamp - last_odom_stamp_).toSec();
        CaptureOdom2dPtr new_capture = std::make_shared<CaptureOdom2d>(
            TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
            sensor_ptr_,
            Eigen::Vector2d(msg->twist.twist.linear.x * dt, msg->twist.twist.angular.z * dt),
            Eigen::DiagonalMatrix<double, 2>(msg->twist.twist.linear.x * dt * (double)odometry_translational_cov_factor_,
                                             msg->twist.twist.angular.z * dt * (double)odometry_rotational_cov_factor_));
        sensor_ptr_->process(new_capture);
    }
    last_odom_stamp_ = msg->header.stamp;

    ROS_INFO("WolfNodePolyline::odomCallback: end");
    ROS_DEBUG("WolfNodePolyline::odomCallback: end");
}

}  // namespace wolf
