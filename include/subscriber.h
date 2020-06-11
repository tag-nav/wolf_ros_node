#ifndef WOLF_SUBSCRIBER_H_
#define WOLF_SUBSCRIBER_H_

/**************************
 *      WOLF includes     *
 **************************/
#include <core/sensor/sensor_base.h>
#include "factory_subscriber.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace wolf {
WOLF_PTR_TYPEDEFS(Subscriber);

/*
 * Macro for defining Autoconf subscriber creator for WOLF's high level API.
 *
 * Place a call to this macro inside your class declaration (in the subscriber_class.h file),
 * preferably just after the constructors.
 *
 * In order to use this macro, the derived subscriber class, SubscriberClass,
 * must have a constructor available with the API:
 *
 *   SubscriberClass(const std::string& _unique_name,
 *                   const ParamsServer& _server,
 *                   const SensorBasePtr _sensor_ptr);
 */
#define WOLF_SUBSCRIBER_CREATE(SubscriberClass)                                     \
static SubscriberPtr create(const std::string& _unique_name,                        \
                            const ParamsServer& _server,                            \
                            const SensorBasePtr _sensor_ptr)                        \
{                                                                                   \
    return std::make_shared<SubscriberClass>(_unique_name, _server, _sensor_ptr);   \
}                                                                                   \

class Subscriber
{
    protected:
        //wolf
        SensorBasePtr sensor_ptr_;
        std::string prefix_;

        // ros
        ros::Subscriber sub_;

    public:
        Subscriber(const std::string& _unique_name,
                   const ParamsServer& _server,
                   const SensorBasePtr _sensor_ptr) :
            sensor_ptr_(_sensor_ptr),
            prefix_("ROS subscriber/" + _unique_name)
        {
        }
        virtual ~Subscriber(){};

        virtual void initSubscriber(ros::NodeHandle& nh, const std::string& topic) = 0;
};
}
#endif
