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
#include "sensor_msgs/JointState.h"

#include "subscriber.h"

namespace wolf
{
class SubscriberDiffdrive : public Subscriber
{
   protected:
        ros::Time last_odom_stamp_;
        Eigen::Vector2d last_angles_;
        int last_odom_seq_;
        int last_kf = -1;
        double ticks_cov_factor_;

   public:

    SubscriberDiffdrive(const std::string& _unique_name,
                        const ParamsServer& _server,
                        const SensorBasePtr _sensor_ptr);
    WOLF_SUBSCRIBER_CREATE(SubscriberDiffdrive);

    virtual void initSubscriber(ros::NodeHandle& nh, const std::string& topic);

    void callback(const sensor_msgs::JointState::ConstPtr& msg);
};

WOLF_REGISTER_SUBSCRIBER(SubscriberDiffdrive)
}  // namespace wolf
