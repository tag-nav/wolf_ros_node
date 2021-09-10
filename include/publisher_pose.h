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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

namespace wolf
{

class PublisherPose: public Publisher
{
        bool pose_array_, marker_, pose_with_cov_;
        bool extrinsics_;
        int max_points_;
        double line_size_;

        geometry_msgs::PoseArray pose_array_msg_;
        visualization_msgs::Marker marker_msg_;
        geometry_msgs::PoseWithCovarianceStamped pose_with_cov_msg_;
        std_msgs::ColorRGBA marker_color_;
        SensorBasePtr sensor_;
        std::string frame_id_, map_frame_id_;

        ros::Publisher pub_pose_array_, pub_marker_, pub_pose_with_cov_;

    public:
        PublisherPose(const std::string& _unique_name,
                      const ParamsServer& _server,
                      const ProblemPtr _problem);
        WOLF_PUBLISHER_CREATE(PublisherPose);

        virtual ~PublisherPose(){};

        void initialize(ros::NodeHandle &nh, const std::string& topic) override;

        void publishDerived() override;

        void publishPose();

    protected:

        bool listenTf();
        Eigen::Quaterniond q_frame_;
        Eigen::Vector3d t_frame_;
        tf::TransformListener tfl_;
};

WOLF_REGISTER_PUBLISHER(PublisherPose)
}

#endif
