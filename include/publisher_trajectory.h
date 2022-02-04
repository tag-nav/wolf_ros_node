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
#ifndef PUBLISHER_TRAJECTORY_H
#define PUBLISHER_TRAJECTORY_H

/**************************
 *      WOLF includes     *
 **************************/
#include "core/problem/problem.h"

#include "publisher.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

namespace wolf
{

class PublisherTrajectory: public Publisher
{
        bool extrinsics_;
        int max_points_;
        double line_size_;

        nav_msgs::Path path_msg_;
        nav_msgs::Odometry odometry_msg_;

        visualization_msgs::Marker marker_msg_;
        std_msgs::ColorRGBA marker_color_;
        SensorBasePtr sensor_;
        std::string frame_id_, map_frame_id_;

        ros::Publisher pub_path_, pub_marker_, pub_odometry_;

    public:
        PublisherTrajectory(const std::string& _unique_name,
                      const ParamsServer& _server,
                      const ProblemPtr _problem);
        WOLF_PUBLISHER_CREATE(PublisherTrajectory);

        virtual ~PublisherTrajectory(){};

        void initialize(ros::NodeHandle &nh, const std::string& topic) override;

        void publishDerived() override;

        void publishTrajectory();

    protected:

        bool listenTf();
        Eigen::Quaterniond q_frame_;
        Eigen::Vector3d t_frame_;
        tf::TransformListener tfl_;
};

WOLF_REGISTER_PUBLISHER(PublisherTrajectory)
}

#endif
