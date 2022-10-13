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

#include "subscriber_landmarks.h"

/**************************
 *      WOLF includes     *
 **************************/
#include <core/capture/capture_landmarks_external.h>
#include <core/math/covariance.h>

/**************************
 *      ROS includes      *
 **************************/
#include <tf/transform_datatypes.h>

namespace wolf
{
SubscriberLandmarks::SubscriberLandmarks(const std::string& _unique_name,
                                         const ParamsServer& _server,
                                         const SensorBasePtr _sensor_ptr)
  : Subscriber(_unique_name, _server, _sensor_ptr)
{
    assert(_sensor_ptr);
    dim = _sensor_ptr->getProblem()->getDim();
}

void SubscriberLandmarks::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    sub_ = nh.subscribe(topic, 100, &SubscriberLandmarks::callback, this);
}

void SubscriberLandmarks::callback(const wolf_ros_node::LandmarkDetectionArray::ConstPtr& msg)
{
    ROS_INFO("SubscriberLandmarks::callback: %lu detections", msg->detections.size());

    updateLastHeader(msg->header);

    auto cap = std::make_shared<CaptureLandmarksExternal>(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec), sensor_ptr_);
    // Extract detections from msg
    for (auto i = 0; i < msg->detections.size(); i++)
    {
        // measurement
        VectorXd meas(dim == 2 ? 3 : 7);
        if (dim == 2)
        {
            meas(0) = msg->detections.at(i).pose.pose.position.x;
            meas(1) = msg->detections.at(i).pose.pose.position.y;
            meas(2) = tf::getYaw(msg->detections.at(i).pose.pose.orientation);
        }
        else
        {
            meas(0) = msg->detections.at(i).pose.pose.position.x;
            meas(1) = msg->detections.at(i).pose.pose.position.y;
            meas(2) = msg->detections.at(i).pose.pose.position.z;
            meas(3) = msg->detections.at(i).pose.pose.orientation.x;
            meas(4) = msg->detections.at(i).pose.pose.orientation.y;
            meas(5) = msg->detections.at(i).pose.pose.orientation.z;
            meas(6) = msg->detections.at(i).pose.pose.orientation.w;
        }
        // covariance
        // PoseWithCovariance documentation:
        //   Row-major representation of the 6x6 covariance matrix.
        //   The orientation parameters use a fixed-axis representation.
        //   In order, the parameters are: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        MatrixXd cov(dim == 2 ? 3 : 6, dim == 2 ? 3 : 6);
        if (dim == 2)
        {
            cov << msg->detections.at(i).pose.covariance.at(0),  msg->detections.at(i).pose.covariance.at(1),  msg->detections.at(i).pose.covariance.at(5),
                   msg->detections.at(i).pose.covariance.at(6),  msg->detections.at(i).pose.covariance.at(7),  msg->detections.at(i).pose.covariance.at(11),
                   msg->detections.at(i).pose.covariance.at(30), msg->detections.at(i).pose.covariance.at(31), msg->detections.at(i).pose.covariance.at(35);
        }
        else
            cov = Eigen::Map<const Eigen::Matrix6d>(msg->detections.at(i).pose.covariance.data());
        
        std::cout << "\tid " << msg->detections.at(i).id << ": quality: " << msg->detections.at(i).quality << ", meas: " << meas.transpose() << std::endl;

        // fill capture
        makePosDef(cov);
        cap->addDetection(msg->detections.at(i).id, meas, cov, msg->detections.at(i).quality);
    }

    // process
    sensor_ptr_->process(cap);

    ROS_DEBUG("SubscriberLandmarks::callback: end");
}

WOLF_REGISTER_SUBSCRIBER(SubscriberLandmarks)
}  // namespace wolf
