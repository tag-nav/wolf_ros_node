//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------

#include "subscriber.h"
#include "subscriber_odom2d.h"

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

    updateLastHeader(msg->header);

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

WOLF_REGISTER_SUBSCRIBER(SubscriberOdom2d)
}  // namespace wolf
