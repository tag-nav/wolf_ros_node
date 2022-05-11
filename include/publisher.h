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
#ifndef WOLF_PUBLISHER_H
#define WOLF_PUBLISHER_H
/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>

/**************************
 *      WOLF includes     *
 **************************/
#include "core/common/wolf.h"
#include "core/problem/problem.h"
#include "factory_publisher.h"

namespace wolf
{
WOLF_PTR_TYPEDEFS(Publisher);

/*
 * Macro for defining Autoconf publisher creator for WOLF's high level API.
 *
 * Place a call to this macro inside your class declaration (in the publisher_class.h file),
 * preferably just after the constructors.
 *
 * In order to use this macro, the derived publisher class, PublisherClass,
 * must have a constructor available with the API:
 *
 *   PublisherClass(const std::string& _unique_name,
 *                  const ParamsServer& _server,
 *                  const ProblemPtr _problem);
 */
#define WOLF_PUBLISHER_CREATE(PublisherClass)                                               \
        static PublisherPtr create(const std::string& _unique_name,                         \
                                   const ParamsServer& _server,                             \
                                   ProblemConstPtr _problem,                                \
                                   ros::NodeHandle& _nh)                                    \
                                   {                                                        \
    PublisherPtr pub = std::make_shared<PublisherClass>(_unique_name, _server, _problem);   \
    pub->initialize(_nh, pub->getTopic());                                                  \
    return pub;                                                                             \
}                                                                                           \

class Publisher
{
    public:

        Publisher(const std::string& _unique_name,
                  const ParamsServer& _server,
                  ProblemConstPtr _problem) :
                      problem_(_problem),
                      first_publish_time_(ros::Time(0)),
                      last_n_period_(0),
                      name_(_unique_name),
                      prefix_("ROS publisher/" + _unique_name),
                      n_publish_(0),
                      acc_duration_(0),
                      max_duration_(0)
        {
            period_ = _server.getParam<double>(prefix_ + "/period");
            topic_  = _server.getParam<std::string>(prefix_ + "/topic");
        };

        virtual ~Publisher(){};

        virtual void run() final;
        virtual void stop() final;
        virtual void loop() final;

        virtual void initialize(ros::NodeHandle& nh, const std::string& topic) = 0;

        virtual void publish() final;

        virtual void publishDerived() = 0;

        virtual bool ready();

        std::string getTopic() const;

        std::string getName() const;

    protected:

        template<typename T>
        T getParamWithDefault(const ParamsServer &_server,
                              const std::string &_param_name,
                              const T _default_value) const;

        ProblemConstPtr problem_;
        ros::Publisher publisher_;
        double period_;
        ros::Time first_publish_time_;
        long unsigned int last_n_period_;
        std::string name_;
        std::string prefix_;
        std::string topic_;

        std::thread pub_thread_;

        // PROFILING
        unsigned int n_publish_;
        std::chrono::microseconds acc_duration_;
        std::chrono::microseconds max_duration_;

    public:
        void printProfiling(std::ostream& stream = std::cout) const;
};

inline std::string Publisher::getTopic() const
{
    return topic_;
}

inline std::string Publisher::getName() const
{
    return name_;
}

inline void Publisher::publish()
{
    if (last_n_period_ == 0)
        first_publish_time_ = ros::Time::now();

    auto start = std::chrono::high_resolution_clock::now();

    publishDerived();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
    acc_duration_+= duration;
    max_duration_ = std::max(max_duration_,duration);
    n_publish_++;
}

inline bool Publisher::ready()
{
    return true;
}

inline void Publisher::printProfiling(std::ostream &_stream) const
{
    _stream << "\n" << name_ << ":"
            << "\n\ttotal time:           " << 1e-6 * acc_duration_.count() << " s"
            << "\n\texecutions:           " << n_publish_
            << "\n\taverage time:         " << 1e-3 * acc_duration_.count() / n_publish_ << " ms"
            << "\n\tmax time:             " << 1e-3 * max_duration_.count() << " ms" << std::endl;
}

inline void Publisher::run()
{
    pub_thread_ = std::thread(&Publisher::loop, this);
}

inline void Publisher::stop()
{
    pub_thread_.join();
}

inline void Publisher::loop()
{
    auto awake_time = std::chrono::system_clock::now();
    auto period = std::chrono::duration<int,std::milli>((int)(period_*1e3));

    WOLF_DEBUG("Started publisher ", name_, " loop");

    while (ros::ok())
    {
        if (ready())
            publish();

        // relax to fit output rate
        awake_time += period;
        std::this_thread::sleep_until(awake_time);
    }
    WOLF_DEBUG("Publisher ", name_, " loop finished");
}

template<typename T>
inline T Publisher::getParamWithDefault(const ParamsServer &_server,
                                        const std::string &_param_name,
                                        const T _default_value) const
{
    try
    {
        return _server.getParam<T>(_param_name);
    }
    catch (...)
    {
        WOLF_INFO("Publisher: Parameter ", _param_name, " is missing. Taking default value: ", _default_value);
        return _default_value;
    }
}

}  // namespace wolf
#endif
