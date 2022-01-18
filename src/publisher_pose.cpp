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
#include "publisher_pose.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"

namespace wolf
{

PublisherPose::PublisherPose(const std::string& _unique_name,
                             const ParamsServer& _server,
                             const ProblemPtr _problem) :
        Publisher(_unique_name, _server, _problem)
{
    Eigen::Vector4d marker_color_v;
    marker_color_v = getParamWithDefault<Eigen::Vector4d>(_server,
                                                          prefix_ + "/marker_color",
                                                          (Eigen::Vector4d() << 1, 0, 0, 1).finished()); // red
    marker_color_.r = marker_color_v(0);
    marker_color_.g = marker_color_v(1);
    marker_color_.b = marker_color_v(2);
    marker_color_.a = marker_color_v(3);

    max_points_ = getParamWithDefault<int>(_server,
                                           prefix_ + "/max_points",
                                           1e4);
    line_size_  = getParamWithDefault<double>(_server,
                                              prefix_ + "/line_size",
                                              0.1);

    extrinsics_ = _server.getParam<bool>(prefix_ + "/extrinsics");
    if (extrinsics_)
        sensor_ = _problem->getSensor(_server.getParam<std::string>(prefix_ + "/sensor"));
    frame_id_ = _server.getParam<std::string>(prefix_ + "/frame_id");
}

void PublisherPose::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    std::string map_frame_id;
    nh.param<std::string>("map_frame_id", map_frame_id_, "map");

    // initialize msg and publisher

    // POSE ARRAY
    pose_array_msg_.header.frame_id = frame_id_;

    pub_pose_array_ = nh.advertise<geometry_msgs::PoseArray>(topic + "_pose_array", 1);

    // MARKER
    marker_msg_.header.frame_id = frame_id_;
    marker_msg_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_msg_.action = visualization_msgs::Marker::ADD;
    marker_msg_.ns = "trajectory";
    marker_msg_.scale.x = line_size_;
    marker_msg_.color = marker_color_;
    marker_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    pub_marker_ = nh.advertise<visualization_msgs::Marker>(topic + "_marker", 1);

    // POSE WITH COV
    pose_with_cov_msg_.header.frame_id = frame_id_;

    pub_pose_with_cov_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic + "_pose_with_cov", 1);
}

void PublisherPose::publishDerived()
{
    if (pub_pose_array_.getNumSubscribers() == 0 and
        pub_marker_.getNumSubscribers() == 0 and
        pub_pose_with_cov_.getNumSubscribers() == 0 )
        return;

    VectorComposite current_state = problem_->getState("PO");
    TimeStamp loc_ts = problem_->getTimeStamp();

    // state not ready
    if (current_state.count('P') == 0 or
        current_state.count('O') == 0 or
        not loc_ts.ok())
    {
        return;
    }

    // fill vector and quaternion
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q;

    // 2D
    if (problem_->getDim() == 2)
    {
        if (extrinsics_)
        {
            p.head(2) = current_state['P'] + Eigen::Rotation2Dd(current_state['O'](0)) * sensor_->getP()->getState().head(2);
            if (sensor_->getO())
                q = Eigen::Quaterniond(Eigen::AngleAxisd(current_state['O'](0) + sensor_->getO()->getState()(0),
                                                         Eigen::Vector3d::UnitZ()));
            else
                q = Eigen::Quaterniond(Eigen::AngleAxisd(current_state['O'](0),
                                                         Eigen::Vector3d::UnitZ()));
        }
        else
        {
            p.head(2) = current_state['P'];
            q = Eigen::Quaterniond(Eigen::AngleAxisd(current_state['O'](0), Eigen::Vector3d::UnitZ()));
        }
    }
    // 3D
    else
    {
        if (extrinsics_)
        {
            p = current_state['P'] + Eigen::Quaterniond(Eigen::Vector4d(current_state['O'])) * sensor_->getP()->getState();
            if (sensor_->getO())
                q = Eigen::Quaterniond(Eigen::Vector4d(current_state['O'])) * Eigen::Quaterniond(Eigen::Vector4d(sensor_->getO()->getState()));
            else
                q = Eigen::Quaterniond(Eigen::Vector4d(current_state['O']));
        }
        else
        {
            p = current_state['P'];
            q = Eigen::Quaterniond(Eigen::Vector4d(current_state['O']));
        }
    }

    // Change frame
    if (frame_id_ != map_frame_id_ and listenTf())
    {
        p = t_frame_ + q_frame_ * p;
        q = q_frame_ * q;
    }

    // Covariance
    Eigen::MatrixXd cov(6,6);
    auto KF = problem_->getLastFrame();
    bool success(true);
    success = success && problem_->getCovarianceBlock(KF->getP(), KF->getP(), cov, 0, 0);
    success = success && problem_->getCovarianceBlock(KF->getP(), KF->getO(), cov, 0, 3);
    success = success && problem_->getCovarianceBlock(KF->getO(), KF->getP(), cov, 3, 0);
    success = success && problem_->getCovarianceBlock(KF->getO(), KF->getO(), cov, 3, 3);

    if (success)
    {
        if (problem_->getDim() == 2)
            throw std::runtime_error("not implemented");
        else
            std::copy(cov.data(), cov.data() + cov.size(), pose_with_cov_msg_.pose.covariance.data());
    }
    else
    {
        //WOLF_WARN("Last KF covariance could not be recovered, using the previous one");
        //pose_with_cov_msg_.pose.covariance[0] = -1; // not valid
    }

    // Fill Pose msg
    pose_with_cov_msg_.header.stamp = ros::Time(loc_ts.getSeconds(), loc_ts.getNanoSeconds());
    pose_with_cov_msg_.pose.pose.position.x = p(0);
    pose_with_cov_msg_.pose.pose.position.y = p(1);
    pose_with_cov_msg_.pose.pose.position.z = p(2);

    pose_with_cov_msg_.pose.pose.orientation.x = q.x();
    pose_with_cov_msg_.pose.pose.orientation.y = q.y();
    pose_with_cov_msg_.pose.pose.orientation.z = q.z();
    pose_with_cov_msg_.pose.pose.orientation.w = q.w();
    publishPose();
}

void PublisherPose::publishPose()
{
    // fill msgs and publish
    if (pub_pose_array_.getNumSubscribers() != 0)
    {
        pose_array_msg_.header.stamp = pose_with_cov_msg_.header.stamp;

        if (max_points_ >= 0 and pose_array_msg_.poses.size() >= max_points_)
        {
            int i = 1;
            while (i < pose_array_msg_.poses.size())
            {
                pose_array_msg_.poses.erase(pose_array_msg_.poses.begin()+i);
                i++;
            }
            //pose_array_msg_.poses.erase(pose_array_msg_.poses.begin(),
            //                            pose_array_msg_.poses.begin() + max_points_/2);
        }

        pose_array_msg_.poses.push_back(pose_with_cov_msg_.pose.pose);

        pub_pose_array_.publish(pose_array_msg_);
    }
    if (pub_marker_.getNumSubscribers() != 0)
    {
        marker_msg_.header.stamp = pose_with_cov_msg_.header.stamp;

        if (max_points_ >= 0 and marker_msg_.points.size() >= max_points_)
        {
            auto it = marker_msg_.points.begin();
            std::advance(it,1);
            while (it != marker_msg_.points.end())
            {
                it = marker_msg_.points.erase(it);
                if (it == marker_msg_.points.end())
                    break;
                std::advance(it,1);
            }
            //int i = 1;
            //while (i < marker_msg_.points.size())
            //{
            //    marker_msg_.points.erase(marker_msg_.points.begin()+i);
            //    i++;
            //}
            //marker_msg_.points.erase(marker_msg_.points.begin(),
            //                         marker_msg_.points.begin() + max_points_/2);
        }

        marker_msg_.points.push_back(pose_with_cov_msg_.pose.pose.position);

        pub_marker_.publish(marker_msg_);
    }
    if (pub_pose_with_cov_.getNumSubscribers() != 0)
    {
        pub_pose_with_cov_.publish(pose_with_cov_msg_);
    }
}

bool PublisherPose::listenTf()
{
    tf::StampedTransform T;
    if ( tfl_.waitForTransform(frame_id_, map_frame_id_, ros::Time(0), ros::Duration(0.01)) )
    {
        tfl_.lookupTransform(frame_id_, map_frame_id_, ros::Time(0), T);

        Eigen::Matrix3d R;
        tf::matrixTFToEigen(T.getBasis(), R);
        tf::vectorTFToEigen(T.getOrigin(), t_frame_);
        q_frame_ = Eigen::Quaterniond(R);

        return true;
    }
    return false;
}

}
