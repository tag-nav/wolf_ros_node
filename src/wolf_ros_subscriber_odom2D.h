/**************************
 *      WOLF includes     *
 **************************/
#include <core/common/wolf.h>
#include <core/utils/params_server.hpp>

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


#include "wolf_ros_subscriber.h"
#include "subscriber_factory.h"

namespace wolf
{
class SubscriberWrapperOdom2D : public SubscriberWrapper
{
   protected:
      ros::Time last_odom_stamp_;
      double odometry_translational_cov_factor_, odometry_rotational_cov_factor_;

   public:

    SubscriberWrapperOdom2D(const SensorBasePtr& sensor_ptr);

    virtual void initSubscriber(ros::NodeHandle& nh, const std::string& topic);

    void callback(const nav_msgs::Odometry::ConstPtr& msg);

    static std::shared_ptr<SubscriberWrapper> create(const std::string& _unique_name, const ParamsServer& _params, const SensorBasePtr _sensor_ptr);
};

WOLF_REGISTER_SUBSCRIBER(SubscriberWrapperOdom2D)
}  // namespace wolf