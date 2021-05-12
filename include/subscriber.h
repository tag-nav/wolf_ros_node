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
#define WOLF_SUBSCRIBER_CREATE(SubscriberClass)                                                 \
static SubscriberPtr create(const std::string& _unique_name,                                    \
                            const ParamsServer& _server,                                        \
                            const SensorBasePtr _sensor_ptr,                                    \
                            ros::NodeHandle& _nh)                                               \
{                                                                                               \
    SubscriberPtr sub = std::make_shared<SubscriberClass>(_unique_name, _server, _sensor_ptr);  \
    sub->initialize(_nh, sub->getTopic());                                                      \
    return sub;                                                                                 \
}                                                                                               \

class Subscriber
{
    protected:
        //wolf
        SensorBasePtr sensor_ptr_;
        std::string prefix_;
        std::string name_;
        std::string topic_;

        // ros
        ros::Subscriber sub_;
        ros::Time last_stamp_;

    public:
        Subscriber(const std::string& _unique_name,
                   const ParamsServer& _server,
                   const SensorBasePtr _sensor_ptr) :
            sensor_ptr_(_sensor_ptr),
            prefix_("ROS subscriber/" + _unique_name),
            name_(_unique_name),
            last_stamp_(0)
        {
            topic_  = _server.getParam<std::string>(prefix_ + "/topic");
        }

        virtual ~Subscriber(){};

        virtual void initialize(ros::NodeHandle& nh, const std::string& topic) = 0;

        std::string getTopic() const;

        std::string getName() const;

        ros::Time getLastStamp() const;

        virtual double secondsSinceLastCallback();

    protected:

        void setLastStamp(const ros::Time _stamp);
};

inline std::string Subscriber::getTopic() const
{
    return topic_;
}

inline std::string Subscriber::getName() const
{
    return name_;
}

inline ros::Time Subscriber::getLastStamp() const
{
    return last_stamp_;
}

inline double Subscriber::secondsSinceLastCallback()
{
    if (last_stamp_ == ros::Time(0))
    {
        WOLF_WARN("Subscriber: 'last_stamp_` not initialized. No messages have been received or ", name_, " is not updating this attribute.");
        return 0;
    }
    return (ros::Time::now() - last_stamp_).toSec();
}

inline void Subscriber::setLastStamp(const ros::Time _stamp)
{
    last_stamp_ = _stamp;
}

}
#endif
