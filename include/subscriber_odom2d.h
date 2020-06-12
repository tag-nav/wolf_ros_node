/**************************
 *      WOLF includes     *
 **************************/
#include <core/common/wolf.h>
#include <core/utils/params_server.h>

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "subscriber.h"

namespace wolf
{
class SubscriberOdom2d : public Subscriber
{
   protected:
      ros::Time last_odom_stamp_;
      double odometry_translational_cov_factor_, odometry_rotational_cov_factor_;

   public:

    SubscriberOdom2d(const std::string& _unique_name,
                     const ParamsServer& _server,
                     const SensorBasePtr _sensor_ptr);
    WOLF_SUBSCRIBER_CREATE(SubscriberOdom2d);

    virtual void initialize(ros::NodeHandle& nh, const std::string& topic);

    void callback(const nav_msgs::Odometry::ConstPtr& msg);
};

WOLF_REGISTER_SUBSCRIBER(SubscriberOdom2d)
}  // namespace wolf
