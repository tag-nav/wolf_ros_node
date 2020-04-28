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
#include "factory_subscriber.h"

namespace wolf
{
class SubscriberOdom2d : public Subscriber
{
   protected:
      ros::Time last_odom_stamp_;
      double odometry_translational_cov_factor_, odometry_rotational_cov_factor_;

   public:

    SubscriberOdom2d(const SensorBasePtr& sensor_ptr);

    virtual void initSubscriber(ros::NodeHandle& nh, const std::string& topic);

    void callback(const nav_msgs::Odometry::ConstPtr& msg);

    static std::shared_ptr<Subscriber> create(const std::string& _unique_name, const ParamsServer& _params, const SensorBasePtr _sensor_ptr);
};

WOLF_REGISTER_SUBSCRIBER(SubscriberOdom2d)
}  // namespace wolf
