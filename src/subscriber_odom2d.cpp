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
  , sensor_odom_(std::static_pointer_cast<SensorOdom2d>(_sensor_ptr))
{
    assert(std::dynamic_pointer_cast<SensorOdom2d>(_sensor_ptr) != nullptr && "SubscriberOdom2d: sensor provided is not of type SensorOdom2d!");
}

void SubscriberOdom2d::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    sub_ = nh.subscribe(topic, 100, &SubscriberOdom2d::callback, this);
}

void SubscriberOdom2d::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_DEBUG("WolfNodePolyline::odomCallback");

    if (last_odom_stamp_ != ros::Time(0))
    {
        double           dt          = (msg->header.stamp - last_odom_stamp_).toSec();
        Eigen::Vector2d data(msg->twist.twist.linear.x * dt, msg->twist.twist.angular.z * dt);
        CaptureOdom2dPtr new_capture = std::make_shared<CaptureOdom2d>(
            TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
            sensor_ptr_,
            data,
            sensor_odom_->computeCovFromMotion(data));
        sensor_ptr_->process(new_capture);
    }
    last_odom_stamp_ = msg->header.stamp;

    ROS_DEBUG("WolfNodePolyline::odomCallback: end");
}

}  // namespace wolf
