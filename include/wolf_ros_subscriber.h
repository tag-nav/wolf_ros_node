#ifndef WOLF_ROS_SUBSCRIBER_H_
#define WOLF_ROS_SUBSCRIBER_H_

/**************************
 *      WOLF includes     *
 **************************/
#include <core/sensor/sensor_base.h>

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace wolf {
WOLF_PTR_TYPEDEFS(SubscriberWrapper);

class SubscriberWrapper
{
    protected:
        //wolf
        SensorBasePtr sensor_ptr_;

        // ros
        ros::Subscriber sub_;

    public:
        SubscriberWrapper(const SensorBasePtr& sensor_ptr) :
            sensor_ptr_(sensor_ptr)
        {
        }
        virtual ~SubscriberWrapper(){};

        virtual void initSubscriber(ros::NodeHandle& nh, const std::string& topic) = 0;
};
}
#endif
