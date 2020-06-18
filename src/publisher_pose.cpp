#include "publisher_pose.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include "tf/transform_datatypes.h"

namespace wolf
{

PublisherPose::PublisherPose(const std::string& _unique_name,
                             const ParamsServer& _server,
                             const ProblemPtr _problem) :
        Publisher(_unique_name, _server, _problem)
{
    pose_array_ = _server.getParam<bool>(prefix_ + "/pose_array_msg");
    marker_     = _server.getParam<bool>(prefix_ + "/marker_msg");
    if (marker_)
    {
        Eigen::Vector4d col = _server.getParam<Eigen::Vector4d>(prefix_ + "/marker_color");
        marker_color_.r = col(0);
        marker_color_.g = col(1);
        marker_color_.b = col(2);
        marker_color_.a = col(3);
    }
}

void PublisherPose::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    std::string map_frame_id;
    nh.param<std::string>("map_frame_id", map_frame_id, "map");

    // initialize msg and publisher
    if (pose_array_)
    {
        pose_array_msg_.header.frame_id = map_frame_id;

        pub_pose_array_ = nh.advertise<geometry_msgs::PoseArray>(topic + "_pose_array", 1);
    }
    if (marker_)
    {
        marker_msg_.header.frame_id = map_frame_id;
        marker_msg_.type = visualization_msgs::Marker::LINE_STRIP;
        marker_msg_.action = visualization_msgs::Marker::ADD;
        marker_msg_.ns = "trajectory";
        marker_msg_.scale.x = 0.1;
        marker_msg_.color = marker_color_;
        marker_msg_.pose.orientation = tf::createQuaternionMsgFromYaw(0);

        pub_marker_ = nh.advertise<visualization_msgs::Marker>(topic + "_marker", 1);
    }
}

void PublisherPose::publishDerived()
{
    VectorComposite current_state = problem_->getState("PO");
    TimeStamp loc_ts = problem_->getTimeStamp();

    // state not ready
    if (current_state.count('P') == 0 or
        current_state.count('O') == 0 or
        loc_ts == TimeStamp(0))
    {
        return;
    }

    // Fill Pose msg
    geometry_msgs::Pose pose_msg;

    // 2D
    if (problem_->getDim() == 2)
    {
        pose_msg.position.x = current_state['P'](0);
        pose_msg.position.y = current_state['P'](1);
        pose_msg.position.z = 0;

        pose_msg.orientation = tf::createQuaternionMsgFromYaw(current_state['O'](0));
    }
    // 3D
    else
    {
        pose_msg.position.x = current_state['P'](0);
        pose_msg.position.y = current_state['P'](1);
        pose_msg.position.z = current_state['P'](2);

        pose_msg.orientation.x = current_state['O'](0);
        pose_msg.orientation.y = current_state['O'](1);
        pose_msg.orientation.z = current_state['O'](2);
        pose_msg.orientation.w = current_state['O'](3);
    }

    publishPose(pose_msg, ros::Time(loc_ts.getSeconds(), loc_ts.getNanoSeconds()));
}

void PublisherPose::publishPose(const geometry_msgs::Pose pose, const ros::Time& stamp)
{
    // fill msgs and publish
    if (pose_array_)
    {
        pose_array_msg_.header.stamp = stamp;
        pose_array_msg_.poses.push_back(pose);

        pub_pose_array_.publish(pose_array_msg_);
    }
    if (marker_)
    {
        marker_msg_.header.stamp = stamp;
        marker_msg_.points.push_back(pose.position);

        pub_marker_.publish(marker_msg_);
    }
}

}
