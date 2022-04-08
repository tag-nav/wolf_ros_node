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
#ifndef PUBLISHER_TF_H
#define PUBLISHER_TF_H

/**************************
 *      WOLF includes     *
 **************************/
#include "core/problem/problem.h"

#include "publisher.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace wolf
{

tf::Transform stateToTfTransform(const VectorComposite& state, const int& dim)
{
    assert(state.includesStructure("PO"));

    // 2D
    if (dim == 2)
    {
        return tf::Transform (tf::createQuaternionFromYaw(state.at('O')(0)),
                              tf::Vector3(state.at('P')(0), state.at('P')(1), 0) );
    }
    // 3D
    else
    {
        return tf::Transform (tf::Quaternion(state.at('O')(0), state.at('O')(1), state.at('O')(2), state.at('O')(3)),
                              tf::Vector3(state.at('P')(0), state.at('P')(1), state.at('P')(2)) );
    }
}

class PublisherTf: public Publisher
{
    protected:
        std::string base_frame_id_, odom_frame_id_, map_frame_id_;
        tf::TransformBroadcaster tfb_;
        tf2_ros::StaticTransformBroadcaster stfb_;

        tf::TransformListener tfl_;

        tf::StampedTransform T_odom2base_;
        geometry_msgs::TransformStamped Tmsg_map2odom_;

        bool publish_odom_tf_;
        bool state_available_; // used to not repeat warnings regarding availability of state

    public:
        PublisherTf(const std::string& _unique_name,
                      const ParamsServer& _server,
                      const ProblemPtr _problem);
        WOLF_PUBLISHER_CREATE(PublisherTf);

        virtual ~PublisherTf(){};

        void initialize(ros::NodeHandle &nh, const std::string& topic) override;

        void publishDerived() override;
};

}

#endif
