#ifndef PUBLISHER_STATE_BLOCK_H
#define PUBLISHER_STATE_BLOCK_H

/**************************
 *      WOLF includes     *
 **************************/
#include "core/problem/problem.h"

#include "publisher.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace wolf
{

class PublisherStateBlock: public Publisher
{
    protected:
        std_msgs::Float64MultiArray state_msg_;
        SensorBasePtr sensor_;
        char key_;
        bool msg_init_;

    public:
        PublisherStateBlock(const std::string& _unique_name,
                      const ParamsServer& _server,
                      const ProblemPtr _problem);
        WOLF_PUBLISHER_CREATE(PublisherStateBlock);

        virtual ~PublisherStateBlock(){};

        void initialize(ros::NodeHandle &nh, const std::string& topic) override;

        void publishDerived() override;
};

WOLF_REGISTER_PUBLISHER(PublisherStateBlock)
}

#endif
