#ifndef PUBLISHER_POSE_H
#define PUBLISHER_POSE_H

/**************************
 *      WOLF includes     *
 **************************/
#include "core/problem/problem.h"

#include "factory_publisher.h"
#include "publisher.h"

namespace wolf
{

class PublisherPose: public Publisher
{
        std::string map_frame_id_;

    public:
        PublisherPose(const std::string& _unique_name,
                      const ParamsServer& _server);
        WOLF_PUBLISHER_CREATE(PublisherPose);
        virtual ~PublisherPose(){};
        void initialize(ros::NodeHandle &nh, const std::string& topic) override;
        void publish(const ProblemPtr problem) override;
        static std::shared_ptr<Publisher> create();
};

WOLF_REGISTER_PUBLISHER(PublisherPose)
}

#endif
