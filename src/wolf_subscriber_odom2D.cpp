/**************************
 *      WOLF includes     *
 **************************/
#include <core/capture/capture_odom_2D.h>
#include <core/sensor/sensor_odom_2D.h>
#include <core/processor/processor_odom_2D.h>

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

#include "wolf_subscriber.h"
#include "wolf_subscriber_odom2D.h"

namespace wolf
{
SubscriberOdom2D::SubscriberOdom2D(const SensorBasePtr& sensor_ptr)
  : WolfSubscriber(sensor_ptr)
  , last_odom_stamp_(ros::Time(0))
  , odometry_translational_cov_factor_(std::static_pointer_cast<SensorOdom2D>(sensor_ptr)->getDispVarToDispNoiseFactor())
  , odometry_rotational_cov_factor_(std::static_pointer_cast<SensorOdom2D>(sensor_ptr)->getRotVarToRotNoiseFactor())
{
}

void SubscriberOdom2D::initSubscriber(ros::NodeHandle& nh, const std::string& topic)
{
    sub_ = nh.subscribe(topic, 100, &SubscriberOdom2D::callback, this);
}

void SubscriberOdom2D::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_DEBUG("WolfNodePolyline::odomCallback");
    ROS_INFO("WolfNodePolyline::odomCallback: start");

    if (last_odom_stamp_ != ros::Time(0))
    {
        double           dt          = (msg->header.stamp - last_odom_stamp_).toSec();
        CaptureOdom2DPtr new_capture = std::make_shared<CaptureOdom2D>(
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

    std::shared_ptr<WolfSubscriber> SubscriberOdom2D::create(const std::string&  _unique_name,
                                                                       const ParamsServer& _params,
                                                                       const SensorBasePtr _sensor_ptr)
    {
        return std::make_shared<SubscriberOdom2D>(_sensor_ptr);
    };
}  // namespace wolf
