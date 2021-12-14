//--------LICENSE_START--------
//
// Copyright (C) 2020,2021 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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
#include "publisher_tf.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"

namespace wolf
{

PublisherTf::PublisherTf(const std::string& _unique_name,
                         const ParamsServer& _server,
                         const ProblemPtr _problem) :
        Publisher(_unique_name, _server, _problem),
        state_available_(true)
{
    map_frame_id_    = _server.getParam<std::string>(prefix_ + "/map_frame_id");
    odom_frame_id_   = _server.getParam<std::string>(prefix_ + "/odom_frame_id");
    base_frame_id_   = _server.getParam<std::string>(prefix_ + "/base_frame_id");
    publish_odom_tf_ = _server.getParam<bool>(prefix_ + "/publish_odom_tf");

    // initialize TF transforms
    T_odom2base_.setIdentity();
    T_odom2base_.frame_id_ = odom_frame_id_;
    T_odom2base_.child_frame_id_ = base_frame_id_;
    T_odom2base_.stamp_ = ros::Time::now();
    //T_map2odom_.setIdentity();
    //T_map2odom_.frame_id_ = map_frame_id_;
    //T_map2odom_.child_frame_id_ = odom_frame_id_;
    //T_map2odom_.stamp_ = ros::Time::now();
    Tmsg_map2odom_.child_frame_id = odom_frame_id_;
    Tmsg_map2odom_.header.frame_id = map_frame_id_;
    Tmsg_map2odom_.header.stamp = ros::Time::now();
}

void PublisherTf::initialize(ros::NodeHandle& nh, const std::string& topic)
{
}

void PublisherTf::publishDerived()
{
    // get current state and stamp
    auto current_state = problem_->getState("PO");
    auto current_ts = problem_->getTimeStamp();

    // MAP - BASE
    //Get map2base from wolf result, and builds base2map pose
    tf::Transform T_map2base;
    if (current_state.count('P') == 0 or
        current_state.count('O') == 0 or
        not current_ts.ok())
    {
        if (state_available_)
        {
            ROS_WARN("PublisherTf: State not available...");
            state_available_ = false; // warning won't be displayed again
        }
        T_map2base.setIdentity();
    }
    else
    {
        if (not state_available_)
        {
            ROS_INFO("PublisherTf: State available!");
            state_available_ = true; // warning won't be displayed again
        }
        T_map2base = stateToTfTransform(current_state, problem_->getDim());
    }

    // MAP - ODOM
    // Wolf odometry
    if (publish_odom_tf_)
    {
        ros::Time transform_time;
        std::string error_msg;
        if (tfl_.getLatestCommonTime(odom_frame_id_, base_frame_id_, transform_time, &error_msg) == tf::ErrorValues::NO_ERROR and
            transform_time != T_odom2base_.stamp_)
        {
            ROS_WARN("PublisherTf: option 'publish_odom_tf' enabled but a transform between %s and %s was found published by a third party. Changing 'publish_odom_tf' to false.",
                     base_frame_id_.c_str(),
                     odom_frame_id_.c_str());

            tfl_.lookupTransform(odom_frame_id_, base_frame_id_, ros::Time(0), T_odom2base_);

            publish_odom_tf_=false;

        }
        else
        {
            VectorComposite odom = problem_->getOdometry("PO");

            T_odom2base_.setData(stateToTfTransform(odom, problem_->getDim()));
            T_odom2base_.stamp_ = ros::Time::now();
            tfb_.sendTransform(T_odom2base_);
        }
    }
    // TF odometry
    else
    {
        try
        {
            tfl_.lookupTransform(odom_frame_id_, base_frame_id_, ros::Time(0), T_odom2base_);
        }
        catch(...)
        {
            ROS_WARN("No %s to %s frame received. Assuming identity.",
                     base_frame_id_.c_str(),
                     odom_frame_id_.c_str());
            T_odom2base_.setIdentity();
            T_odom2base_.stamp_ = ros::Time::now();
        }
    }


    // Broadcast transform ---------------------------------------------------------------------------
    //T_map2odom_.setData(T_map2base * T_odom2base_.inverse());
    //T_map2odom_.stamp_ = ros::Time::now();
    //std::cout << "T_map2odom: " << T_map2odom.getOrigin().getX() << " " << T_map2odom.getOrigin().getY() << " " << T_map2odom.getRotation().getAngle() << std::endl;
    //tfb_.sendTransform(T_map2odom_);

    tf::Transform T_map2odom = T_map2base * T_odom2base_.inverse();

    Tmsg_map2odom_.transform.translation.x = T_map2odom.getOrigin().getX();
    Tmsg_map2odom_.transform.translation.y = T_map2odom.getOrigin().getY();
    Tmsg_map2odom_.transform.translation.z = T_map2odom.getOrigin().getZ();

    Tmsg_map2odom_.transform.rotation.x = T_map2odom.getRotation().getX();
    Tmsg_map2odom_.transform.rotation.y = T_map2odom.getRotation().getY();
    Tmsg_map2odom_.transform.rotation.z = T_map2odom.getRotation().getZ();
    Tmsg_map2odom_.transform.rotation.w = T_map2odom.getRotation().getW();

    Tmsg_map2odom_.header.stamp = ros::Time::now();

    stfb_.sendTransform(Tmsg_map2odom_);
}

}
