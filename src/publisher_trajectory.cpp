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
/*
 * publisher_trajectory.cpp
 *
 *  Created on: Feb 03, 2022
 *      Author: igeer
 */

#include "publisher_trajectory.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>

namespace wolf
{

PublisherTrajectory::PublisherTrajectory(const std::string& _unique_name,
                                         const ParamsServer& _server,
                                         ProblemConstPtr _problem) :
        Publisher(_unique_name, _server, _problem)
{
    frame_id_ = _server.getParam<std::string>(prefix_ + "/frame_id");
}

void PublisherTrajectory::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    nh.param<std::string>("frame_id", frame_id_, "map");

    // initialize msg and publisher

    // PATH
    // path_msg_.header.frame_id = frame_id_;
    // publisher_ = nh.advertise<nav_msgs::Path>(topic, 1);

    pose_msg_.header.frame_id = frame_id_;
    publisher_ = nh.advertise<geometry_msgs::PoseStamped>(topic, 1);
}

void PublisherTrajectory::publishDerived()
{
    if (publisher_.getNumSubscribers() != 0 )
        publishTrajectory();
}

void PublisherTrajectory::publishTrajectory()
{
    pose_msg_.header.frame_id = frame_id_;
    // pose_msg_.header.stamp = ros::Time::now();

    auto frame_map = problem_->getTrajectory()->getFrameMap();
    if (!frame_map.empty())
    {
        auto last_frame = std::prev(frame_map.end()); // Get the iterator to the last element

        auto loc_ts = last_frame->first;
        pose_msg_.header.stamp = ros::Time(loc_ts.getSeconds(), loc_ts.getNanoSeconds());

        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q;

        if (problem_->getDim() == 2)
        {
            p.head(2) = last_frame->second->getP()->getState();
            q = Eigen::Quaterniond(Eigen::AngleAxisd(last_frame->second->getO()->getState()(0), Eigen::Vector3d::UnitZ()));
        }
        else
        {
            p = last_frame->second->getP()->getState();
            q = Eigen::Quaterniond(Eigen::Vector4d(last_frame->second->getO()->getState()));
        }

        pose_msg_.pose.position.x = p(0);
        pose_msg_.pose.position.y = p(1);
        pose_msg_.pose.position.z = p(2);
        pose_msg_.pose.orientation.x = q.x();
        pose_msg_.pose.orientation.y = q.y();
        pose_msg_.pose.orientation.z = q.z();
        pose_msg_.pose.orientation.w = q.w();

        // Publish pose
        publisher_.publish(pose_msg_);
    }
}

WOLF_REGISTER_PUBLISHER(PublisherTrajectory)
}
