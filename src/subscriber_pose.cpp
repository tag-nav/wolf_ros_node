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

#include "subscriber_pose.h"

/**************************
 *      WOLF includes     *
 **************************/

/**************************
 *      ROS includes      *
 **************************/

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <iomanip>
#include <queue>


namespace wolf
{
SubscriberPose::SubscriberPose(const std::string& _unique_name,
                                   const ParamsServer& _server,
                                   const SensorBasePtr _sensor_ptr)
  : Subscriber(_unique_name, _server, _sensor_ptr)
  , last_pose_stamp_(ros::Time(0))
  , sensor_pose_(std::static_pointer_cast<SensorPose>(_sensor_ptr))
{
    assert(std::dynamic_pointer_cast<SensorPose>(_sensor_ptr) != nullptr && "SubscriberPose: sensor provided is not of type SensorPose!");
}

void SubscriberPose::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    sub_ = nh.subscribe(topic, 100, &SubscriberPose::callback, this);
}

void SubscriberPose::callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    updateLastHeader(msg->header);

    if (last_pose_stamp_ != ros::Time(0))
    {
        double           dt          = (msg->header.stamp - last_pose_stamp_).toSec();
        Eigen::Vector7d data;
        data << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
        CapturePosePtr new_capture = std::make_shared<CapturePose>(
            TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
            sensor_ptr_,
            data,
            sensor_pose_->getNoiseCov());
        sensor_ptr_->process(new_capture);
    }
    last_pose_stamp_ = msg->header.stamp;

}

WOLF_REGISTER_SUBSCRIBER(SubscriberPose)
}  // namespace wolf
