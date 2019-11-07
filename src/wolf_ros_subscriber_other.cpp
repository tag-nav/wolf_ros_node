/**************************
 *      WOLF includes     *
 **************************/
#include <core/capture/capture_odom_2D.h>
#include <core/sensor/sensor_odom_2D.h>
#include <core/processor/processor_odom_2D.h>
#include <core/yaml/parser_yaml.hpp>
#include <core/common/wolf.h>
#include <core/problem/problem.h>
#include <core/utils/params_server.hpp>

/**************************
 *     CERES includes     *
 **************************/
#include <core/ceres_wrapper/ceres_manager.h>
//#include "glog/logging.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <iomanip>
#include <queue>

#include "wolf_ros_subscriber.h"

using namespace wolf;

class WolfSubscriberWrapperOther: public WolfSubscriberWrapper
{
   protected:
      ros::Time last_odom_stamp_;
      double odometry_translational_cov_factor_, odometry_rotational_cov_factor_;

   public:

      WolfSubscriberWrapperOther(const SensorBasePtr& sensor_ptr) :
         WolfSubscriberWrapper(sensor_ptr),
         last_odom_stamp_(ros::Time(0)),
         odometry_translational_cov_factor_(std::static_pointer_cast<SensorOdom2D>(sensor_ptr)->getDispVarToDispNoiseFactor()),
         odometry_rotational_cov_factor_(std::static_pointer_cast<SensorOdom2D>(sensor_ptr)->getRotVarToRotNoiseFactor())
      {
      }

      virtual void initSubscriber(ros::NodeHandle& nh, const std::string& topic)
      {
         sub_ = nh.subscribe(topic,100,&WolfSubscriberWrapperOther::callback,this);
      }

      void callback(const nav_msgs::Odometry::ConstPtr& msg)
      {
         ROS_DEBUG("WolfNodePolyline::odomCallback");
         ROS_INFO("Other callback: start");

         if (last_odom_stamp_ != ros::Time(0))
         {
            Scalar dt = (msg->header.stamp - last_odom_stamp_).toSec();
            CaptureOdom2DPtr new_capture = std::make_shared<CaptureOdom2D>(TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec),
                                                                           sensor_ptr_,
                                                                           Eigen::Vector2s(msg->twist.twist.linear.x*dt,
                                                                                           msg->twist.twist.angular.z*dt),
                                                                           Eigen::DiagonalMatrix<Scalar,2>(msg->twist.twist.linear.x*dt*(Scalar)odometry_translational_cov_factor_,
                                                                                                           msg->twist.twist.angular.z*dt*(Scalar)odometry_rotational_cov_factor_));
            sensor_ptr_->process(new_capture);
         }
         last_odom_stamp_ = msg->header.stamp;

         ROS_INFO("Other callback: end");
         ROS_DEBUG("WolfNodePolyline::odomCallback: end");
      }

    static std::shared_ptr<WolfSubscriberWrapper> create(const std::string& _unique_name, const ParamsServer& _params, const SensorBasePtr _sensor_ptr)
    {
        return std::make_shared<WolfSubscriberWrapperOther>(_sensor_ptr);
    }
};
#include "subscriber_factory.h"
WOLF_REGISTER_SUBSCRIBER(WolfSubscriberWrapperOther)