#include "publisher_tf.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"

namespace wolf
{

PublisherTf::PublisherTf(const std::string& _unique_name,
                             const ParamsServer& _server,
                             const ProblemPtr _problem) :
        Publisher(_unique_name, _server, _problem),
        state_available_(true)
{
    map_frame_id_   = _server.getParam<std::string>(prefix_ + "/map_frame_id");
    odom_frame_id_  = _server.getParam<std::string>(prefix_ + "/odom_frame_id");
    base_frame_id_  = _server.getParam<std::string>(prefix_ + "/base_frame_id");
}

void PublisherTf::initialize(ros::NodeHandle& nh, const std::string& topic)
{
}

void PublisherTf::publishDerived()
{
    // get current state and stamp
    auto current_state = problem_->getState("PO");
    auto current_ts = problem_->getTimeStamp();

    //Get map2base from wolf result, and builds base2map pose
    tf::Transform T_map2base;
    if (current_state.count('P') == 0 or
        current_state.count('O') == 0 or
        not current_ts.ok())
    {
        if (state_available_)
        {
            ROS_WARN("PublisherTf: State not available...");
            state_available_ = false; // warning won't be displayed again
        }
        T_map2base.setIdentity();
    }
    else
    {
        if (not state_available_)
        {
            ROS_INFO("PublisherTf: State available!");
            state_available_ = true; // warning won't be displayed again
        }
        // 2D
        if (problem_->getDim() == 2)
        {
            T_map2base = tf::Transform (tf::createQuaternionFromYaw(current_state['O'](0)),
                                        tf::Vector3(current_state['P'](0),
                                                    current_state['P'](1),
                                                    0) );
        }
        // 3D
        else
        {
            T_map2base = tf::Transform (tf::Quaternion(current_state['O'](0),
                                                       current_state['O'](1),
                                                       current_state['O'](2),
                                                       current_state['O'](3)),
                                        tf::Vector3(current_state['P'](0),
                                                    current_state['P'](1),
                                                    current_state['P'](2)) );
        }
    }

    //gets T_map2odom_ (odom wrt map), by using tf listener, and assuming an odometry node is broadcasting odom2base
    tf::StampedTransform T_base2odom;
    if ( tfl_.waitForTransform(base_frame_id_, odom_frame_id_, ros::Time(0), ros::Duration(0.2)) )
    {
        tfl_.lookupTransform(base_frame_id_, odom_frame_id_, ros::Time(0), T_base2odom);
        //std::cout << ros::Time::now().sec << " Odometry: " << T_base2odom.inverse().getOrigin().getX() << " " << T_base2odom.inverse().getOrigin().getY() << " " << T_base2odom.inverse().getRotation().getAngle() << std::endl;
    }
    else
    {
        ROS_WARN("No %s to %s frame received", base_frame_id_.c_str(), odom_frame_id_.c_str());
        T_base2odom.setIdentity();
        T_base2odom.frame_id_ = base_frame_id_;
        T_base2odom.child_frame_id_ = odom_frame_id_;
        T_base2odom.stamp_ = ros::Time::now();
    }

    // Broadcast transform ---------------------------------------------------------------------------
    tf::StampedTransform T_map2odom(T_map2base * T_base2odom,
                                    ros::Time::now(),
                                    map_frame_id_,
                                    odom_frame_id_);
    //std::cout << "T_map2odom: " << T_map2odom.getOrigin().getX() << " " << T_map2odom.getOrigin().getY() << " " << T_map2odom.getRotation().getAngle() << std::endl;
    tfb_.sendTransform(T_map2odom);
}

}
