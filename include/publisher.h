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

namespace wolf
{
WOLF_PTR_TYPEDEFS(Publisher);

/*
 * Macro for defining Autoconf subscriber creator for WOLF's high level API.
 *
 * Place a call to this macro inside your class declaration (in the subscriber_class.h file),
 * preferably just after the constructors.
 *
 * In order to use this macro, the derived subscriber class, PublisherClass,
 * must have a constructor available with the API:
 *
 *   PublisherClass(const std::string& _unique_name,
 *                   const ParamsServer& _server,
 *                   const SensorBasePtr _sensor_ptr);
 */
#define WOLF_PUBLISHER_CREATE(PublisherClass)                                    \
static PublisherPtr create(const std::string& _unique_name,                      \
                            const ParamsServer& _server)                         \
{                                                                                \
    return std::make_shared<PublisherClass>(_unique_name, _server); \
}                                                                                \

class Publisher
{
  public:

    Publisher(const std::string& _unique_name,
              const ParamsServer& _server)
      : period_(1)
      , last_publish_time_(ros::Time(0))
    {};

    virtual ~Publisher(){};

    virtual void initialize(ros::NodeHandle& nh, const std::string& topic) = 0;

    virtual void publish(const ProblemPtr problem) = 0;

    double period_;
    ros::Time last_publish_time_;

  protected:

    ros::Publisher publisher_;
};
}  // namespace wolf
#endif
