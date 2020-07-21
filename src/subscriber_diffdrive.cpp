/**************************
 *      WOLF includes     *
 **************************/
#include <core/capture/capture_diff_drive.h>
#include <core/sensor/sensor_diff_drive.h>

#include "core/math/rotations.h"
#include "subscriber_diffdrive.h"

namespace wolf
{
SubscriberDiffdrive::SubscriberDiffdrive(const std::string& _unique_name,
                                         const ParamsServer& _server,
                                         const SensorBasePtr _sensor_ptr)
  : Subscriber(_unique_name, _server, _sensor_ptr)
  , last_odom_stamp_(ros::Time(0))
  , last_odom_seq_(-1)
{
  last_angles_ = Eigen::Vector2d();
  ticks_cov_factor_ = std::static_pointer_cast<SensorDiffDrive>(_sensor_ptr)->getParams()->ticks_cov_factor;
}

void SubscriberDiffdrive::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    sub_ = nh.subscribe(topic, 100, &SubscriberDiffdrive::callback, this);
}

void SubscriberDiffdrive::callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  auto left_angle            = msg->position[0];
  auto right_angle           = msg->position[1];
  auto msg_angles            = Eigen::Vector2d(left_angle, right_angle);
  double ticks_per_revolution = 360;

  Eigen::Vector2d angles_inc (pi2pi(msg_angles(0) - last_angles_(0)), pi2pi(msg_angles(1) - last_angles_(1)));
  angles_inc *= ticks_per_revolution/(2*M_PI);

  Eigen::MatrixXd cov        = ticks_cov_factor_ * (angles_inc.cwiseAbs() + Eigen::Vector2d::Ones()).asDiagonal();  // TODO check this

  if (last_odom_seq_ != -1)
  {
      CaptureDiffDrivePtr cptr = std::make_shared<CaptureDiffDrive>(
          TimeStamp(msg->header.stamp.sec, msg->header.stamp.nsec), sensor_ptr_, angles_inc, cov, nullptr);
      sensor_ptr_->process(cptr);

      auto current_kf = sensor_ptr_->getProblem()->getLastFrame()->id();

      if(last_kf != current_kf)
      {
      // sensor_ptr_->getProblem()->print(4,0,1,1);

//       std::cout << "\n===========================================" << std::endl;
//       auto const capture_origin = std::static_pointer_cast<CaptureMotion>(sensor_ptr_->getProblem()->getProcessorMotion()->getOrigin());
// //      auto const capture_last = std::static_pointer_cast<CaptureMotion>(std::static_pointer_cast<ProcessorDiffDrive>(sensor_ptr_->getProblem()->getProcessorMotion())->getLast());
//       // capture_origin->getBuffer().print(1,1,1,1);
//       std::cout << "===========================================\n" << std::endl;

      last_kf = current_kf;
      }

  }

  last_angles_     = msg_angles;
  last_odom_stamp_ = msg->header.stamp;
  last_odom_seq_   = msg->header.seq;
}

}  // namespace wolf
