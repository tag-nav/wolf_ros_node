#ifndef WOLF_ROS_PUBLISHER_H
#define WOLF_ROS_PUBLISHER_H
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
WOLF_PTR_TYPEDEFS(WolfRosPublisher);
class WolfRosPublisher
{
  public:

    WolfRosPublisher()
      : period_(1)
      , last_publish_time_(ros::Time(0))
    {};

    virtual ~WolfRosPublisher(){};

    virtual void initialize(ros::NodeHandle& nh, const std::string& topic) = 0;

    virtual void publish(const ProblemPtr problem) = 0;

    double period_;
    ros::Time last_publish_time_;

  protected:

    ros::Publisher publisher_;
};
}  // namespace wolf
#endif
