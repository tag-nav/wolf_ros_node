#include "publisher_pose.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>

namespace wolf
{

PublisherPose::PublisherPose() :
    Publisher()
{
}

void PublisherPose::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    publisher_ = nh.advertise<nav_msgs::Odometry>(topic, 1);
    nh.param<std::string>("map_frame_id", map_frame_id_, "map");
}

void PublisherPose::publish(const ProblemPtr _problem)
{
    VectorComposite current_state = _problem->getState("PO");
    TimeStamp loc_ts = _problem->getTimeStamp();

    // state not ready
    if (current_state.count("P") == 0 or
        current_state.count("O") == 0 or
        loc_ts == TimeStamp(0))
    {
        return;
    }

    // Fill PoseStamped msg
    nav_msgs::Odometry msg;
    msg.header.frame_id = map_frame_id_;
    msg.header.stamp = ros::Time(loc_ts.getSeconds(), loc_ts.getNanoSeconds());

    // 2D
    if (_problem->getDim() == 2)
    {
        msg.pose.pose.position.x = current_state["P"](0);
        msg.pose.pose.position.y = current_state["P"](1);
        msg.pose.pose.position.z = 0;

        msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_state["O"](0));
    }
    // 3D
    else
    {
        msg.pose.pose.position.x = current_state["P"](0);
        msg.pose.pose.position.y = current_state["P"](1);
        msg.pose.pose.position.z = current_state["P"](2);

        msg.pose.pose.orientation.x = current_state["O"](0);
        msg.pose.pose.orientation.y = current_state["O"](1);
        msg.pose.pose.orientation.z = current_state["O"](2);
        msg.pose.pose.orientation.w = current_state["O"](3);
    }

    publisher_.publish(msg);
}

std::shared_ptr<Publisher> PublisherPose::create()
{
    return std::make_shared<PublisherPose>();
}

}
