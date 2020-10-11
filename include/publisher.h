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
                                   const ProblemPtr _problem,                               \
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
                  const ProblemPtr _problem) :
                      problem_(_problem),
                      first_publish_time_(ros::Time(0)),
                      n_published_(0),
                      prefix_("ROS publisher/" + _unique_name)
        {
            period_ = _server.getParam<double>(prefix_ + "/period");
            topic_  = _server.getParam<std::string>(prefix_ + "/topic");
        };

        virtual ~Publisher(){};

        virtual void initialize(ros::NodeHandle& nh, const std::string& topic) = 0;

        virtual void publish() final;

        virtual void publishDerived() = 0;

        virtual bool ready();

        std::string getTopic() const;

    protected:

        ProblemPtr problem_;
        ros::Publisher publisher_;
        double period_;
        ros::Time first_publish_time_;
        long unsigned int n_published_;
        std::string prefix_;
        std::string topic_;
};

inline void Publisher::publish()
{
    if (n_published_ == 0)
        first_publish_time_ = ros::Time::now();

    n_published_++;
    publishDerived();
}

inline bool Publisher::ready()
{
    long unsigned int n_pub = (long unsigned int)((ros::Time::now() - first_publish_time_).toSec() / period_);
    return n_pub > n_published_;
}

inline std::string Publisher::getTopic() const
{
    return topic_;
}

}  // namespace wolf
#endif
