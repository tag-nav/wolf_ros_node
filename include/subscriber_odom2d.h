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

#ifndef WOLF_SUBSCRIBER_ODOM2D_H_
#define WOLF_SUBSCRIBER_ODOM2D_H_


#include "subscriber.h"

/**************************
 *      WOLF includes     *
 **************************/
#include <core/common/wolf.h>
#include <core/utils/params_server.h>
#include <core/sensor/sensor_odom_2d.h>

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


namespace wolf
{
class SubscriberOdom2d : public Subscriber
{
   protected:
      ros::Time last_odom_stamp_;
      SensorOdom2dPtr sensor_odom_;

   public:

    SubscriberOdom2d(const std::string& _unique_name,
                     const ParamsServer& _server,
                     const SensorBasePtr _sensor_ptr);
    WOLF_SUBSCRIBER_CREATE(SubscriberOdom2d);

    virtual void initialize(ros::NodeHandle& nh, const std::string& topic);

    void callback(const nav_msgs::Odometry::ConstPtr& msg);
};

}  // namespace wolf

#endif
