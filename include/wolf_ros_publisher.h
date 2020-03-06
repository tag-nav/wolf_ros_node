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
class WolfRosPublisher
{
  public:

    WolfRosPublisher(){};

    virtual ~WolfRosPublisher(){};

    virtual void initialize(ros::NodeHandle& nh) = 0;

    virtual void publish(const ProblemPtr problem) = 0;

  protected:

    ros::Publisher publisher_;
};
}  // namespace wolf
#endif
