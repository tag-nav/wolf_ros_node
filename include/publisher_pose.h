#ifndef PUBLISHER_POSE_H
#define PUBLISHER_POSE_H

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
#include <visualization_msgs/Marker.h>

namespace wolf
{

class PublisherPose: public Publisher
{
        bool pose_array_, marker_;

        geometry_msgs::PoseArray pose_array_msg_;
        visualization_msgs::Marker marker_msg_;
        std_msgs::ColorRGBA marker_color_;

        ros::Publisher pub_pose_array_, pub_marker_;

    public:
        PublisherPose(const std::string& _unique_name,
                      const ParamsServer& _server,
                      const ProblemPtr _problem);
        WOLF_PUBLISHER_CREATE(PublisherPose);

        virtual ~PublisherPose(){};

        void initialize(ros::NodeHandle &nh, const std::string& topic) override;

        void publishDerived() override;

        void publishPose(const geometry_msgs::Pose pose, const ros::Time& stamp);
};

WOLF_REGISTER_PUBLISHER(PublisherPose)
}

#endif
