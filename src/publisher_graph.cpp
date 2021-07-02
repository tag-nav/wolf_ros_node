#include "publisher_graph.h"
#include <tf/transform_datatypes.h>
#include "core/processor/processor_motion.h"

namespace wolf
{

PublisherGraph::PublisherGraph(const std::string& _unique_name,
                               const ParamsServer& _server,
                               const ProblemPtr _problem) :
                Publisher(_unique_name, _server, _problem)
{
    Eigen::Vector4d color;

    // LOAD PARAMETERS (all optionals) ---------------------------------------------------
    // General
    map_frame_id_           = getParamWithDefault<std::string>(_server, prefix_ + "/map_frame_id", "map");
    viz_overlapped_factors_ = getParamWithDefault<bool>     (_server, prefix_ + "/viz_overlapped_factors", false);
    viz_inactive_factors_   = getParamWithDefault<bool>     (_server, prefix_ + "/viz_inactive_factors_", false);
    text_scale_             = getParamWithDefault<double>   (_server, prefix_ + "/text_scale", 0.5);
    viz_scale_              = getParamWithDefault<double>   (_server, prefix_ + "/viz_scale", 1);

    // landmarks
    landmark_text_z_offset_ = getParamWithDefault<double>   (_server, prefix_ + "/landmark_text_z_offset", 1);
    landmark_width_         = getParamWithDefault<double>   (_server, prefix_ + "/landmark_width", 0.1);
    landmark_length_        = getParamWithDefault<double>   (_server, prefix_ + "/landmark_length", 1);

    // frames
    frame_width_            = getParamWithDefault<double>   (_server, prefix_ + "/frame_width", 0.1);
    frame_length_           = getParamWithDefault<double>   (_server, prefix_ + "/frame_length", 1);
    color = getParamWithDefault<Eigen::Vector4d>(_server,
                                                 prefix_ + "/frame_color",
                                                 (Eigen::Vector4d() << 1, 0.8, 0, 1).finished());
    frame_color_.r = color(0);
    frame_color_.g = color(1);
    frame_color_.b = color(2);
    frame_color_.a = color(3);

    // factors
    factors_width_          = getParamWithDefault<double>   (_server, prefix_ + "/factors_width", 0.02);
    factors_absolute_height_= getParamWithDefault<double>   (_server, prefix_ + "/factors_absolute_height", 2);

    color = getParamWithDefault<Eigen::Vector4d>(_server,
                                                 prefix_ + "/factor_abs_color",
                                                 (Eigen::Vector4d() << 1, 0, 0, 1).finished()); // red
    factor_abs_color_.r = color(0);
    factor_abs_color_.g = color(1);
    factor_abs_color_.b = color(2);
    factor_abs_color_.a = color(3);

    color = getParamWithDefault<Eigen::Vector4d>(_server,
                                                 prefix_ + "/factor_motion_color",
                                                 (Eigen::Vector4d() << 1, 1, 0, 1).finished()); // yellow
    factor_motion_color_.r = color(0);
    factor_motion_color_.g = color(1);
    factor_motion_color_.b = color(2);
    factor_motion_color_.a = color(3);

    color = getParamWithDefault<Eigen::Vector4d>(_server,
                                                 prefix_ + "/factor_loop_color",
                                                 (Eigen::Vector4d() << 0, 1, 0, 1).finished()); // green
    factor_loop_color_.r = color(0);
    factor_loop_color_.g = color(1);
    factor_loop_color_.b = color(2);
    factor_loop_color_.a = color(3);

    color = getParamWithDefault<Eigen::Vector4d>(_server,
                                                 prefix_ + "/factor_lmk_color",
                                                 (Eigen::Vector4d() << 0, 1, 1, 1).finished()); // cyan
    factor_lmk_color_.r = color(0);
    factor_lmk_color_.g = color(1);
    factor_lmk_color_.b = color(2);
    factor_lmk_color_.a = color(3);

    color = getParamWithDefault<Eigen::Vector4d>(_server,
                                                 prefix_ + "/factor_geom_color",
                                                 (Eigen::Vector4d() << 0, 0, 1, 1).finished()); // blue
    factor_geom_color_.r = color(0);
    factor_geom_color_.g = color(1);
    factor_geom_color_.b = color(2);
    factor_geom_color_.a = color(3);

    color = getParamWithDefault<Eigen::Vector4d>(_server,
                                                 prefix_ + "/factor_other_color",
                                                 (Eigen::Vector4d() << 1, 1, 1, 1).finished()); // white
    factor_other_color_.r = color(0);
    factor_other_color_.g = color(1);
    factor_other_color_.b = color(2);
    factor_other_color_.a = color(3);

    // INIT MARKERS ---------------------------------------------------
    // factor markers message
    factor_marker_.type = visualization_msgs::Marker::LINE_LIST;
    factor_marker_.action = visualization_msgs::Marker::ADD;
    factor_marker_.header.frame_id = map_frame_id_;
    factor_marker_.ns = "factors";
    factor_marker_.scale.x = viz_scale_*factors_width_;
    factor_text_marker_ = factor_marker_;
    factor_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    factor_text_marker_.ns = "factors_text";
    factor_text_marker_.color.r = 1;
    factor_text_marker_.color.g = 1;
    factor_text_marker_.color.b = 1;
    factor_text_marker_.color.a = 1;
    factor_text_marker_.scale.x = viz_scale_*text_scale_;
    factor_text_marker_.scale.y = viz_scale_*text_scale_;
    factor_text_marker_.scale.z = viz_scale_*text_scale_;

    // frame markers
    frame_marker_.type = visualization_msgs::Marker::ARROW;
    frame_marker_.header.frame_id = map_frame_id_;
    frame_marker_.ns = "frames";
    frame_marker_.scale.x = viz_scale_*frame_length_;
    frame_marker_.scale.y = viz_scale_*frame_width_;
    frame_marker_.scale.z = viz_scale_*frame_width_;
    frame_marker_.color = frame_color_;
    frame_text_marker_ = frame_marker_;
    frame_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    frame_text_marker_.ns = "frames_text";
    frame_text_marker_.color.r = 1;
    frame_text_marker_.color.g = 1;
    frame_text_marker_.color.b = 1;
    frame_text_marker_.color.a = 1;
    frame_text_marker_.scale.x = viz_scale_*text_scale_;
    frame_text_marker_.scale.y = viz_scale_*text_scale_;
    frame_text_marker_.scale.z = viz_scale_*text_scale_;

    // landmark markers
    landmark_marker_.type = visualization_msgs::Marker::ARROW;
    landmark_marker_.header.frame_id = map_frame_id_;
    landmark_marker_.ns = "landmarks";
    landmark_marker_.scale.x = viz_scale_*landmark_length_;
    landmark_marker_.scale.y = viz_scale_*landmark_width_;
    landmark_marker_.scale.z = viz_scale_*landmark_width_;
    landmark_marker_.color.a = 0.5;
    landmark_text_marker_ = landmark_marker_;
    landmark_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    landmark_text_marker_.ns = "landmarks_text";
    landmark_text_marker_.color.r = 1;
    landmark_text_marker_.color.g = 1;
    landmark_text_marker_.color.b = 1;
    landmark_text_marker_.color.a = 1;
    landmark_text_marker_.scale.z = viz_scale_*text_scale_;
}

void PublisherGraph::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    factors_publisher_      = nh.advertise<visualization_msgs::MarkerArray>(topic + "_factors", 1);
    landmarks_publisher_    = nh.advertise<visualization_msgs::MarkerArray>(topic + "_landmarks", 1);
    trajectory_publisher_   = nh.advertise<visualization_msgs::MarkerArray>(topic + "_trajectory", 1);
}

void PublisherGraph::publishDerived()
{
    if (factors_publisher_.getNumSubscribers() != 0)
        publishFactors();
    if (landmarks_publisher_.getNumSubscribers() != 0)
        publishLandmarks();
    if (trajectory_publisher_.getNumSubscribers() != 0)
        publishTrajectory();
}
void PublisherGraph::publishLandmarks()
{
    // Iterate over all landmarks
    int marker_i = 0;
    auto landmark_marker = landmark_marker_;
    auto landmark_text_marker = landmark_text_marker_;
    auto landmark_list = problem_->getMap()->getLandmarkList();
    for (auto lmk : landmark_list)
    {
        // fill markers
        fillLandmarkMarkers(lmk,landmark_marker, landmark_text_marker);

        // Store landmark marker in marker array
        landmark_marker.id = marker_i;
        landmark_marker.header.stamp = ros::Time::now();

        if (landmarks_marker_array_.markers.size() < marker_i+1)
        {
            landmark_marker.action = visualization_msgs::Marker::ADD;
            landmarks_marker_array_.markers.push_back(landmark_marker);
        }
        else
        {
            landmark_marker.action = visualization_msgs::Marker::MODIFY;
            landmarks_marker_array_.markers[marker_i] = landmark_marker;
        }
        marker_i++;

        // Store text landmark marker in marker array
        landmark_text_marker.id = marker_i;
        landmark_text_marker.header.stamp = ros::Time::now();

        if (landmarks_marker_array_.markers.size() < marker_i+1)
        {
            landmark_text_marker.action = visualization_msgs::Marker::ADD;
            landmarks_marker_array_.markers.push_back(landmark_text_marker);
        }
        else
        {
            landmark_text_marker.action = visualization_msgs::Marker::MODIFY;
            landmarks_marker_array_.markers[marker_i] = landmark_text_marker;
        }
        marker_i++;
    }

    // rest of markers (if any) action: DELETE
    for (auto i = marker_i; i < landmarks_marker_array_.markers.size(); i++)
        landmarks_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;

    // publish marker array
    landmarks_publisher_.publish(landmarks_marker_array_);
}


void PublisherGraph::publishFactors()
{
    // update stamps of generic messages
    factor_marker_.header.stamp = ros::Time::now();
    factor_text_marker_.header.stamp = ros::Time::now();

    // previous
    factors_marker_array_.markers.clear();
    factors_marker_array_.markers.push_back(factor_marker_);
    factors_marker_array_.markers.front().action = visualization_msgs::Marker::DELETEALL;

    // Get a list of factors of the trajectory (discarded all prior factors for extrinsics/intrinsics..)
    FactorBasePtrList fac_list;
    problem_->getTrajectory()->getFactorList(fac_list);

    // reset previously drawn factors
    factors_drawn_.clear();

    // Iterate over the list of factors
    for (auto fac : fac_list)
    {
        if (not viz_inactive_factors_ and fac->getStatus() == FAC_INACTIVE)
            continue;

        auto factor_marker = factor_marker_;
        auto factor_text_marker = factor_text_marker_;

        // markers id
        factor_marker.id = fac->id();
        factor_text_marker.id = fac->id();

        // fill marker
        fillFactorMarker(fac, factor_marker, factor_text_marker);

        // Store marker text in marker array
        factors_marker_array_.markers.push_back(factor_text_marker);

        // avoid drawing overlapped factors markers
        if (not viz_overlapped_factors_)
        {
            std::string fac_str = factorString(fac);

            if (factors_drawn_.count(fac_str) == 0)
            {
                factors_marker_array_.markers.push_back(factor_marker);
                factors_drawn_.emplace(fac_str);
            }
        }
        else
            factors_marker_array_.markers.push_back(factor_marker);
    }

    // publish marker array
    factors_publisher_.publish(factors_marker_array_);
}

void PublisherGraph::publishTrajectory()
{
    // Iterate over the key frames
    int marker_i = 0;
    auto frame_marker = frame_marker_;
    auto frame_text_marker = frame_text_marker_;
    auto trajectory = *problem_->getTrajectory();
    for (auto frm : trajectory)
    {
      // fill marker
      fillFrameMarker(frm, frame_marker, frame_text_marker);

      // Store marker in marker array
      frame_marker.id = marker_i;
      frame_marker.header.stamp = ros::Time::now();

      if (trajectory_marker_array_.markers.size() < marker_i+1)
      {
        frame_marker.action = visualization_msgs::Marker::ADD;
        trajectory_marker_array_.markers.push_back(frame_marker);
      }
      else
      {
        frame_marker.action = visualization_msgs::Marker::MODIFY;
        trajectory_marker_array_.markers[marker_i] = frame_marker;
      }
      marker_i++;

      // Store text marker in marker array
      frame_text_marker.id = marker_i;
      frame_text_marker.header.stamp = ros::Time::now();

      if (trajectory_marker_array_.markers.size() < marker_i + 1) {
        frame_text_marker.action = visualization_msgs::Marker::ADD;
        trajectory_marker_array_.markers.push_back(frame_text_marker);
      } else {
        frame_text_marker.action = visualization_msgs::Marker::MODIFY;
        trajectory_marker_array_.markers[marker_i] = frame_text_marker;
      }
      marker_i++;
    }

    // rest of markers (if any) action: DELETE
    for (auto i = marker_i; i < trajectory_marker_array_.markers.size(); i++)
      trajectory_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;

    // publish marker array
    trajectory_publisher_.publish(trajectory_marker_array_);
}

void PublisherGraph::fillLandmarkMarkers(LandmarkBaseConstPtr lmk,
    visualization_msgs::Marker& lmk_marker,
    visualization_msgs::Marker& lmk_text_marker)
{
  // SHAPE ------------------------------------------------------
  // Position
  //    2d: CYLINDER
  //    3d: SPHERE
    // Pose -> ARROW
    if (lmk->getO() != nullptr)
    {
        landmark_marker_.type = visualization_msgs::Marker::ARROW;
        lmk_marker.scale.x = viz_scale_*landmark_length_;
        lmk_marker.scale.y = viz_scale_*landmark_width_;
        lmk_marker.scale.z = viz_scale_*landmark_width_;
    }
    else if (lmk->getP()->getSize() == 2)
    {
        landmark_marker_.type = visualization_msgs::Marker::CYLINDER;
        lmk_marker.scale.x = viz_scale_*landmark_width_;
        lmk_marker.scale.y = viz_scale_*landmark_width_;
        lmk_marker.scale.z = viz_scale_*landmark_length_;
    }
    else
    {
        landmark_marker_.type = visualization_msgs::Marker::SPHERE;
        lmk_marker.scale.x = viz_scale_*landmark_width_;
        lmk_marker.scale.y = viz_scale_*landmark_width_;
        lmk_marker.scale.z = viz_scale_*landmark_width_;
    }

    // COLOR ------------------------------------------------------
    if (lmk->getHits() > landmark_max_hits_)
        landmark_max_hits_ = lmk->getHits();
    lmk_marker.color.r = (double)lmk->getHits()/landmark_max_hits_;
    lmk_marker.color.g = 0;
    lmk_marker.color.b = 1 - (double)lmk->getHits()/landmark_max_hits_;

    // POSITION & ORIENTATION ------------------------------------------------------
    // position
    lmk_marker.pose.position.x = lmk->getP()->getState()(0);
    lmk_marker.pose.position.y = lmk->getP()->getState()(1);
    if (lmk->getP()->getSize() > 2)
        lmk_marker.pose.position.z = lmk->getP()->getState()(2);

    // orientation
    if (lmk->getO() != nullptr)
    {
        // 3d
        if (lmk->getO()->getSize() > 1)
        {
            lmk_marker.pose.orientation.x = lmk->getO()->getState()(0);
            lmk_marker.pose.orientation.y = lmk->getO()->getState()(1);
            lmk_marker.pose.orientation.z = lmk->getO()->getState()(2);
            lmk_marker.pose.orientation.w = lmk->getO()->getState()(3);
        }
        // 2d
        else
            lmk_marker.pose.orientation = tf::createQuaternionMsgFromYaw(lmk->getO()->getState()(0));
    }

    // TEXT MARKER ------------------------------------------------------
    lmk_text_marker.text = std::to_string(lmk->id());
    lmk_text_marker.pose.position.x = lmk_marker.pose.position.x;
    lmk_text_marker.pose.position.y = lmk_marker.pose.position.y;
    lmk_text_marker.pose.position.z = lmk_marker.pose.position.z + viz_scale_*landmark_text_z_offset_;
}

void PublisherGraph::fillFactorMarker(FactorBaseConstPtr fac,
                                      visualization_msgs::Marker &fac_marker,
                                      visualization_msgs::Marker &fac_text_marker)
{
    geometry_msgs::Point point1, point2;

    // point1 -> frame ------------------------------------------------------
    point1.x = fac->getCapture()->getFrame()->getP()->getState()(0);
    point1.y = fac->getCapture()->getFrame()->getP()->getState()(1);
    if (fac->getProblem()->getDim() == 3)
        point1.z = fac->getCapture()->getFrame()->getP()->getState()(2);
    else
        point1.z = 0;

    // point2 -> other ------------------------------------------------------
    // FRAME
    if (fac->getFrameOther() != nullptr) {

        // special case: Motion from ProcessorMotion
        auto proc_motion = std::dynamic_pointer_cast<ProcessorMotion>(fac->getProcessor());
        if (proc_motion and fac->getCaptureOther())
        {
            // Get state of other
            const auto& x_other = fac->getFrameOther()->getState(proc_motion->getStateStructure());

            // Get most recent motion
            const auto& cap_own = std::static_pointer_cast<CaptureMotion>(fac->getFeature()->getCapture());
            const auto& motion = cap_own->getBuffer().back();

            // Get delta preintegrated up to now
            const auto& delta_preint = motion.delta_integr_;

            // Get calibration preint -- stored in last capture
            const auto& calib_preint = cap_own->getCalibrationPreint();

            VectorComposite state_integrated;
            if ( proc_motion->hasCalibration())
            {
                // Get current calibration -- from other capture
                const auto& calib = proc_motion->getCalibration(fac->getCaptureOther());

                // get Jacobian of delta wrt calibration
                const auto& J_delta_calib = motion.jacobian_calib_;

                // compute delta change
                const auto& delta_step = J_delta_calib * (calib - calib_preint);

                // correct delta // this is (+)
                const auto& delta_corrected = proc_motion->correctDelta(delta_preint, delta_step);

                // compute current state // this is [+]
                proc_motion->statePlusDelta(x_other, delta_corrected, cap_own->getTimeStamp() - fac->getCaptureOther()->getTimeStamp(), state_integrated);

            }
            else
            {
                proc_motion->statePlusDelta(x_other, delta_preint, cap_own->getTimeStamp() - fac->getCaptureOther()->getTimeStamp(), state_integrated);
            }

            // FILL POINTS
            // 1=origin (other)
            point1.x = fac->getFrameOther()->getP()->getState()(0);
            point1.y = fac->getFrameOther()->getP()->getState()(1);
            if (fac->getProblem()->getDim() == 3)
                point1.z = fac->getFrameOther()->getP()->getState()(2);
            else
                point1.z = 0;
            // 2=own
            point2.x = state_integrated.at('P')(0);
            point2.y = state_integrated.at('P')(1);
            if (fac->getProblem()->getDim() == 3)
                point2.z = state_integrated.at('P')(2);
            else
                point2.z = 0;
        }
        else
        {
            point2.x = fac->getFrameOther()->getP()->getState()(0);
            point2.y = fac->getFrameOther()->getP()->getState()(1);
            if (fac->getProblem()->getDim() == 3)
                point2.z = fac->getFrameOther()->getP()->getState()(2);
            else
                point2.z = 0;
        }
    }
    // CAPTURE (with Frame)
    else if (fac->getCaptureOther() != nullptr &&
             fac->getCaptureOther()->getFrame() != nullptr) {
        point2.x = fac->getCaptureOther()->getFrame()->getP()->getState()(0);
        point2.y = fac->getCaptureOther()->getFrame()->getP()->getState()(1);
        if (fac->getProblem()->getDim() == 3)
            point2.z = fac->getCaptureOther()->getFrame()->getP()->getState()(2);
        else
            point2.z = 0;
    }
    // FEATURE (with Frame)
    else if (fac->getFeatureOther() != nullptr &&
            fac->getFeatureOther()->getCapture() != nullptr &&
            fac->getFeatureOther()->getCapture()->getFrame() != nullptr) {
        point2.x = fac->getFeatureOther()->getCapture()->getFrame()->getP()->getState()(0);
        point2.y = fac->getFeatureOther()->getCapture()->getFrame()->getP()->getState()(1);
        if (fac->getProblem()->getDim() == 3)
            point2.z = fac->getFeatureOther()->getCapture()->getFrame()->getP()->getState()(2);
        else
            point2.z = 0;
    }
    // LANDMARK
    else if (fac->getLandmarkOther() != nullptr) {
        point2.x = fac->getLandmarkOther()->getP()->getState()(0);
        point2.y = fac->getLandmarkOther()->getP()->getState()(1);
        if (fac->getProblem()->getDim() == 3)
            point2.z = fac->getLandmarkOther()->getP()->getState()(2);
        else
            point2.z = 0;
    }
    // ABSOLUTE
    else {
        point2 = point1;
        point2.z = point1.z + viz_scale_ * factors_absolute_height_;
    }

    // store points ------------------------------------------------------
    fac_marker.points.push_back(point1);
    fac_marker.points.push_back(point2);

    // colors ------------------------------------------------------
    auto color = frame_color_;
    if (fac->getTopology() == TOP_ABS)
        color = factor_abs_color_;
    if (fac->getTopology() == TOP_MOTION)
        color = factor_motion_color_;
    if (fac->getTopology() == TOP_LOOP)
        color = factor_loop_color_;
    if (fac->getTopology() == TOP_LMK)
        color = factor_lmk_color_;
    if (fac->getTopology() == TOP_GEOM)
        color = factor_geom_color_;
    if (fac->getTopology() == TOP_OTHER)
        color = factor_other_color_;

    // more transparent if inactive
    if (fac->getStatus() == FAC_INACTIVE)
        color.a *= 0.5;

    fac_marker.colors.push_back(color);
    fac_marker.colors.push_back(color);// 2 times because of 2 points
    fac_marker.ns = std::string("factors_"+(fac->getProcessor() ? fac->getProcessor()->getName() : "unnamed_processor"));

    // TEXT MARKER --------------------------------------------------------
    fac_text_marker.text = std::to_string(fac->id());
    fac_text_marker.pose.position.x = (point1.x + point2.x)/(double) 2;
    fac_text_marker.pose.position.y = (point1.y + point2.y)/(double) 2;
    fac_text_marker.pose.position.z = fac_marker.pose.position.z;
    fac_text_marker.ns = std::string("factors_text_"+(fac->getProcessor() ? fac->getProcessor()->getName() : "unnamed_processor"));
}

void PublisherGraph::fillFrameMarker(FrameBaseConstPtr frm,
                                     visualization_msgs::Marker &frm_marker,
                                     visualization_msgs::Marker &frm_text_marker)
{
    // SHAPE ------------------------------------------------------
    // Position-> SPHERE
    // Pose -> ARROW
    if (frm->getO() != nullptr) {
        landmark_marker_.type = visualization_msgs::Marker::ARROW;
        frm_marker.scale.x = viz_scale_*frame_length_;
        frm_marker.scale.y = viz_scale_*frame_width_;
        frm_marker.scale.z = viz_scale_*frame_width_;
    } else {
        landmark_marker_.type = visualization_msgs::Marker::SPHERE;
        frm_marker.scale.x = viz_scale_*frame_width_;
        frm_marker.scale.y = viz_scale_*frame_width_;
        frm_marker.scale.z = viz_scale_*frame_width_;
    }

    // POSITION & ORIENTATION
    // ------------------------------------------------------ position
    frm_marker.pose.position.x = frm->getP()->getState()(0);
    frm_marker.pose.position.y = frm->getP()->getState()(1);
    if (frm->getP()->getSize() > 2)
        frm_marker.pose.position.z = frm->getP()->getState()(2);
    else
        frm_marker.pose.position.z = 0;

    // orientation
    if (frm->getO() != nullptr) {
        // 3d
        if (frm->getO()->getSize() > 1) {
            frm_marker.pose.orientation.x = frm->getO()->getState()(0);
            frm_marker.pose.orientation.y = frm->getO()->getState()(1);
            frm_marker.pose.orientation.z = frm->getO()->getState()(2);
            frm_marker.pose.orientation.w = frm->getO()->getState()(3);
        }
        // 2d
        else
            frm_marker.pose.orientation =
                    tf::createQuaternionMsgFromYaw(frm->getO()->getState()(0));
    }
    // TEXT MARKER --------------------------------------------------------
    frm_text_marker.text = std::to_string(frm->id());
    frm_text_marker.pose.position.x = frm_marker.pose.position.x;
    frm_text_marker.pose.position.y = frm_marker.pose.position.y;
    frm_text_marker.pose.position.z = frm_marker.pose.position.z + viz_scale_*landmark_text_z_offset_;
}

std::string PublisherGraph::factorString(FactorBaseConstPtr fac) const
{
    std::string factor_string;
    factor_string = "F" + std::to_string(fac->getCapture()->getFrame()->id());

    // FRAME
    if (fac->getFrameOther() != nullptr)
        factor_string += "_F" + std::to_string(fac->getFrameOther()->id());
    // CAPTURE (with Frame)
    else if (fac->getCaptureOther() != nullptr &&
             fac->getCaptureOther()->getFrame() != nullptr)
        factor_string += "_C" + std::to_string(fac->getCaptureOther()->id());
    // FEATURE (with Frame)
    else if (fac->getFeatureOther() != nullptr &&
            fac->getFeatureOther()->getCapture() != nullptr &&
            fac->getFeatureOther()->getCapture()->getFrame() != nullptr)
        factor_string += "_f" + std::to_string(fac->getFeatureOther()->id());
    // LANDMARK
    else if (fac->getLandmarkOther() != nullptr)
        factor_string += "_L" + std::to_string(fac->getLandmarkOther()->id());
    // ABSOLUTE (nothing

    // Topology
    factor_string += "_T" + fac->getTopology();

    // Processor
    factor_string += "_P" + (fac->getProcessor() ? std::to_string(fac->getProcessor()->id()) : "NO");

    return factor_string;
}

}
