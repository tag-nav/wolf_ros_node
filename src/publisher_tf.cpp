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
    map_frame_id_    = _server.getParam<std::string>(prefix_ + "/map_frame_id");
    odom_frame_id_   = _server.getParam<std::string>(prefix_ + "/odom_frame_id");
    base_frame_id_   = _server.getParam<std::string>(prefix_ + "/base_frame_id");
    publish_odom_tf_ = _server.getParam<bool>(prefix_ + "/publish_odom_tf");

    // initialize TF transforms
    T_odom2base_.setIdentity();
    T_odom2base_.frame_id_ = odom_frame_id_;
    T_odom2base_.child_frame_id_ = base_frame_id_;
    T_odom2base_.stamp_ = ros::Time::now();
    T_map2odom_.setIdentity();
    T_map2odom_.frame_id_ = map_frame_id_;
    T_map2odom_.child_frame_id_ = odom_frame_id_;
    T_map2odom_.stamp_ = ros::Time::now();
}

void PublisherTf::initialize(ros::NodeHandle& nh, const std::string& topic)
{
}

void PublisherTf::publishDerived()
{
    // get current state and stamp
    auto current_state = problem_->getState("PO");
    auto current_ts = problem_->getTimeStamp();

    // MAP - BASE
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
        T_map2base = stateToTfTransform(current_state, problem_->getDim());
    }

    // MAP - ODOM
    // Wolf odometry
    if (publish_odom_tf_)
    {
        ros::Time transform_time;
        std::string error_msg;
        if (tfl_.getLatestCommonTime(odom_frame_id_, base_frame_id_, transform_time, &error_msg) == tf::ErrorValues::NO_ERROR and
            transform_time != T_odom2base_.stamp_)
        {
            WOLF_ERROR("PublisherTf: option 'publish_odom_tf' enabled but a transform between ",
                       base_frame_id_, " and ", odom_frame_id_,
                       " was found published by a third party. Not publishing this transformation.");
        }
        else
        {
            VectorComposite odom;
            for(auto pm : problem_->getProcessorIsMotionList())
            {
                /*auto p = std::dynamic_pointer_cast<ProcessorBase>(pm);
                if(p != nullptr)
                    WOLF_TRACE("Getting odometry of processor ", p->getName(), ":\n", pm->getOdometry());//*/
                odom = pm->getOdometry();
            }

            T_odom2base_.setData(stateToTfTransform(odom, problem_->getDim()));
            T_odom2base_.stamp_ = ros::Time::now();
            tfb_.sendTransform(T_odom2base_);
        }
    }
    // TF odometry
    else
    {
        try
        {
            tfl_.lookupTransform(odom_frame_id_, base_frame_id_, ros::Time(0), T_odom2base_);
        }
        catch(...)
        {
            ROS_WARN("No %s to %s frame received. Assuming identity.", base_frame_id_.c_str(), odom_frame_id_.c_str());
            T_odom2base_.setIdentity();
            T_odom2base_.stamp_ = ros::Time::now();
        }
    }


    // Broadcast transform ---------------------------------------------------------------------------
    T_map2odom_.setData(T_map2base * T_odom2base_.inverse());
    T_map2odom_.stamp_ = ros::Time::now();
    //std::cout << "T_map2odom: " << T_map2odom.getOrigin().getX() << " " << T_map2odom.getOrigin().getY() << " " << T_map2odom.getRotation().getAngle() << std::endl;
    tfb_.sendTransform(T_map2odom_);
}

}
