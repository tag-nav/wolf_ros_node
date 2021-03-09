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

namespace wolf
{

tf::Transform stateToTfTransform(const VectorComposite& state, const int& dim)
{
    assert(state.includesStateStructure("PO"));

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
        tf::TransformListener tfl_;

        tf::StampedTransform T_odom2base_, T_map2odom_;

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

WOLF_REGISTER_PUBLISHER(PublisherTf)
}

#endif
