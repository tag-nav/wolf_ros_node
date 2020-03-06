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
#include "sensor_msgs/JointState.h"


#include "wolf_ros_subscriber.h"
#include "subscriber_factory.h"

namespace wolf
{
class SubscriberWrapperDiffdrive : public SubscriberWrapper
{
   protected:
        ros::Time last_odom_stamp_;
        Eigen::Vector2d last_angles_;
        int last_odom_seq_;
        int last_kf = -1;
        double ticks_cov_factor_;

   public:

    SubscriberWrapperDiffdrive(const SensorBasePtr& sensor_ptr);

    virtual void initSubscriber(ros::NodeHandle& nh, const std::string& topic);

    void callback(const sensor_msgs::JointState::ConstPtr& msg);

    static std::shared_ptr<SubscriberWrapper> create(const std::string& _unique_name, const ParamsServer& _params, const SensorBasePtr _sensor_ptr);
};

WOLF_REGISTER_SUBSCRIBER(SubscriberWrapperDiffdrive)
}  // namespace wolf
