//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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
                             const ProblemPtr _problem) :
        Publisher(_unique_name, _server, _problem)
{
    frame_id_ = _server.getParam<std::string>(prefix_ + "/frame_id");
}

void PublisherTrajectory::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    nh.param<std::string>("frame_id", frame_id_, "map");

    // initialize msg and publisher

    // PATH
    path_msg_.header.frame_id = frame_id_;
    publisher_ = nh.advertise<nav_msgs::Path>(topic, 1);
}

void PublisherTrajectory::publishDerived()
{
    if (publisher_.getNumSubscribers() != 0 )
        publishTrajectory();
}

void PublisherTrajectory::publishTrajectory()
{
    path_msg_.header.stamp = ros::Time::now();

    auto trajectory = problem_->getTrajectory();
    int frame_num = 0;

    //Fill path message with PoseStamped from trajectory
    geometry_msgs::PoseStamped framepose;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q;

    for (auto frm: trajectory->getFrameMap())
    {
        auto loc_ts = frm.first;
        framepose.header.frame_id = frame_id_;
        framepose.header.stamp = ros::Time(loc_ts.getSeconds(), loc_ts.getNanoSeconds());
        if (problem_->getDim() == 2)
        {
            p.head(2) = frm.second->getP()->getState();
            q = Eigen::Quaterniond(Eigen::AngleAxisd(frm.second->getO()->getState()(0), Eigen::Vector3d::UnitZ()));
        }
        else
        {
            p = frm.second->getP()->getState();
            q = Eigen::Quaterniond(Eigen::Vector4d(frm.second->getO()->getState()));

        }
        framepose.pose.position.x = p(0);
        framepose.pose.position.y = p(1);
        framepose.pose.position.z = p(2);
        framepose.pose.orientation.x = q.x();
        framepose.pose.orientation.y = q.y();
        framepose.pose.orientation.z = q.z();
        framepose.pose.orientation.w = q.w();
        path_msg_.poses.push_back(framepose);
        }

    //Publish path
    publisher_.publish(path_msg_);

    //clear msg
    path_msg_.poses.clear();
}

}
