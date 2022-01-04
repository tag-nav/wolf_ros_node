//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
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
        int last_seq_;

    public:
        Subscriber(const std::string& _unique_name,
                   const ParamsServer& _server,
                   const SensorBasePtr _sensor_ptr) :
            sensor_ptr_(_sensor_ptr),
            prefix_("ROS subscriber/" + _unique_name),
            name_(_unique_name),
            last_stamp_(0),
            last_seq_(-1)
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

        void updateLastHeader(const std_msgs::Header& _header);

        template<typename T>
        T getParamWithDefault(const ParamsServer &_server,
                              const std::string &_param_name,
                              const T _default_value) const;
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
        WOLF_WARN("Subscriber: 'last_stamp_` not initialized. No messages have been received or ", name_, " is not updating headers (be sure to add 'updateLastHeader(msg->header)' in your subscriber callback).");
        return 0;
    }
    return (ros::Time::now() - last_stamp_).toSec();
}

inline void Subscriber::updateLastHeader(const std_msgs::Header& _header)
{
    // stamp
    if ((ros::Time::now() - _header.stamp).toSec() > 1) // in case use_sim_time == false
        last_stamp_ = ros::Time::now();
    else
        last_stamp_ = _header.stamp;

    // seq
    if (last_seq_ >= 0 and _header.seq - last_seq_ > 1)
        ROS_ERROR("Subscriber %s lost %i messages!", name_.c_str(), _header.seq - last_seq_ - 1);
    last_seq_ = _header.seq;
}

template<typename T>
inline T Subscriber::getParamWithDefault(const ParamsServer &_server,
                                         const std::string &_param_name,
                                         const T _default_value) const
{
    try
    {
        return _server.getParam<T>(_param_name);
    }
    catch (...)
    {
        WOLF_INFO("Subscriber: Parameter ", _param_name, " is missing. Taking default value: ", _default_value);
        return _default_value;
    }
}

}
#endif
