#include "wolf_ros_visualizer.h"


WolfRosVisualizer::WolfRosVisualizer() :
    landmark_max_hits_(10),
    last_markers_publish_(0)
{
}


void WolfRosVisualizer::initialize(ros::NodeHandle& nh)
{
    // init publishers ---------------------------------------------------
    factors_publisher_      = nh.advertise<visualization_msgs::MarkerArray>("factors", 1);
    landmarks_publisher_    = nh.advertise<visualization_msgs::MarkerArray>("landmarks", 1);
    trajectory_publisher_   = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 1);

    // Load options ---------------------------------------------------
    nh.param<bool>(         "viz_factors",              viz_factors_,               true);
    nh.param<bool>(         "viz_landmarks",            viz_landmarks_,             true);
    nh.param<bool>(         "viz_trajectory",           viz_trajectory_,            true);
    // viz parameters
    nh.param<std::string>(  "map_frame_name",           map_frame_id_,              "map");
    nh.param<double>(       "factors_width",            factors_width_,             0.02);
    nh.param<double>(       "landmark_text_z_offset",   landmark_text_z_offset_,    1);
    nh.param<double>(       "landmark_length",          landmark_length_,           1);
    nh.param<double>(       "frame_width",              frame_width_,               0.1);
    nh.param<double>(       "frame_length",             frame_length_,              1);
    // colors: active yellow, inactive gray
    double col_R, col_G, col_B, col_A;
    nh.param<double>(       "color_active_r",           col_R, 1.0);
    nh.param<double>(       "color_active_g",           col_G, 0.8);
    nh.param<double>(       "color_active_b",           col_B, 0.0);
    nh.param<double>(       "color_active_a",           col_A, 0.5);
    color_active_.r = col_R;
    color_active_.g = col_G;
    color_active_.b = col_B;
    color_active_.a = col_A;
    nh.param<double>(       "color_inactive_r",           col_R, 0.5);
    nh.param<double>(       "color_inactive_g",           col_G, 0.5);
    nh.param<double>(       "color_inactive_b",           col_B, 0.5);
    nh.param<double>(       "color_inactive_a",           col_A, 0.5);
    color_inactive_.r = col_R;
    color_inactive_.g = col_G;
    color_inactive_.b = col_B;
    color_inactive_.a = col_A;

    // init markers ---------------------------------------------------
    // factor markers message
    factor_marker_.type = visualization_msgs::Marker::LINE_LIST;
    factor_marker_.header.frame_id = map_frame_id_;
    factor_marker_.ns = "/factors";
    factor_marker_.scale.x = factors_width_;

    // frame markers
    frame_marker_.type = visualization_msgs::Marker::ARROW;
    frame_marker_.header.frame_id = map_frame_id_;
    frame_marker_.ns = "/frames";
    frame_marker_.scale.x = frame_length_;
    frame_marker_.scale.y = frame_width_;
    frame_marker_.scale.z = frame_width_;
    frame_marker_.color = color_active_;

    // landmark markers
    landmark_marker_.type = visualization_msgs::Marker::ARROW;
    landmark_marker_.header.frame_id = map_frame_id_;
    landmark_marker_.ns = "/landmarks";
    landmark_marker_.scale.x = landmark_length_;
    landmark_marker_.scale.y = landmark_width_;
    landmark_marker_.scale.z = landmark_width_;
    landmark_marker_.color.a = 0.5;
    landmark_text_marker_ = landmark_marker_;
    landmark_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    landmark_text_marker_.ns = "/landmarks_text";
    landmark_text_marker_.color.r = 1;
    landmark_text_marker_.color.g = 1;
    landmark_text_marker_.color.b = 1;
    landmark_text_marker_.color.a = 0.5;
    landmark_text_marker_.scale.z = 3;
}

void WolfRosVisualizer::visualize(const ProblemPtr problem)
{
    if (viz_factors_)
        publishFactors(problem);
    if (viz_landmarks_)
        publishLandmarks(problem);
    if (viz_trajectory_)
        publishTrajectory(problem);
}

void WolfRosVisualizer::publishLandmarks(const ProblemPtr problem)
{
    // Iterate over all landmarks
    int marker_i = 0;
    auto landmark_marker = landmark_marker_;
    auto landmark_text_marker = landmark_text_marker_;
    for (auto lmk : problem->getMap()->getLandmarkList())
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


void WolfRosVisualizer::publishFactors(const ProblemPtr problem)
{
    // Get a list of factors of the trajectory (discarded all prior factors for extrinsics/intrinsics..)
    FactorBasePtrList fac_list;
    problem->getTrajectory()->getFactorList(fac_list);

    // Iterate over the list of factors
    int marker_i = 0;
    auto factor_marker = factor_marker_;
    for (auto fac : fac_list)
    {
        // fill marker
        fillFactorMarker(fac, factor_marker);

        // Store marker in marker array
        factor_marker.id = marker_i;
        factor_marker.header.stamp = ros::Time::now();

        if (factors_marker_array_.markers.size() < marker_i+1)
        {
            factor_marker.action = visualization_msgs::Marker::ADD;
            factors_marker_array_.markers.push_back(factor_marker);
        }
        else
        {
            factor_marker.action = visualization_msgs::Marker::MODIFY;
            factors_marker_array_.markers[marker_i] = factor_marker;
        }
        marker_i++;
    }

    // rest of markers (if any) action: DELETE
    for (auto i = marker_i; i < factors_marker_array_.markers.size(); i++)
        factors_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;

    // publish marker array
    factors_publisher_.publish(factors_marker_array_);
}

void WolfRosVisualizer::publishTrajectory(const ProblemPtr problem)
{
    // Iterate over the key frames
    int marker_i = 0;
    auto frame_marker = frame_marker_;
    for (auto frm : problem->getTrajectory()->getFrameList())
        if (frm->isKey())
        {
            // fill marker
            fillFrameMarker(frm, frame_marker);

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
        }

    // rest of markers (if any) action: DELETE
    for (auto i = marker_i; i < trajectory_marker_array_.markers.size(); i++)
        trajectory_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;

    // publish marker array
    trajectory_publisher_.publish(trajectory_marker_array_);
}

void WolfRosVisualizer::fillLandmarkMarkers(LandmarkBaseConstPtr lmk,
                                            visualization_msgs::Marker& lmk_marker,
                                            visualization_msgs::Marker& lmk_text_marker)
{
    // SHAPE ------------------------------------------------------
    // Position
    //    2D: CYLINDER
    //    3D: SPHERE
    // Pose -> ARROW
    if (lmk->getO() != nullptr)
    {
        landmark_marker_.type = visualization_msgs::Marker::ARROW;
        lmk_marker.scale.x = landmark_length_;
        lmk_marker.scale.y = landmark_width_;
        lmk_marker.scale.z = landmark_width_;
    }
    else if (lmk->getP()->getSize() == 2)
    {
        landmark_marker_.type = visualization_msgs::Marker::CYLINDER;
        lmk_marker.scale.x = landmark_width_;
        lmk_marker.scale.y = landmark_width_;
        lmk_marker.scale.z = landmark_length_;
    }
    else
    {
        landmark_marker_.type = visualization_msgs::Marker::SPHERE;
        lmk_marker.scale.x = landmark_width_;
        lmk_marker.scale.y = landmark_width_;
        lmk_marker.scale.z = landmark_width_;
    }

    // COLOR ------------------------------------------------------
    if (lmk->getHits() > landmark_max_hits_)
        landmark_max_hits_ = lmk->getHits();
    lmk_marker.color.r = (double)lmk->getHits()/landmark_max_hits_;
    lmk_marker.color.g = 0;
    lmk_marker.color.b = 1 - (double)lmk->getHits()/landmark_max_hits_;
    lmk_marker.color.a = 0.5;

    // POSITION & ORIENTATION ------------------------------------------------------
    // position
    lmk_marker.pose.position.x = lmk->getP()->getState()(0);
    lmk_marker.pose.position.y = lmk->getP()->getState()(1);
    if (lmk->getP()->getSize() > 2)
        lmk_marker.pose.position.z = lmk->getP()->getState()(2);

    // orientation
    if (lmk->getO() != nullptr)
    {
        // 3D
        if (lmk->getO()->getSize() > 1)
        {
            lmk_marker.pose.orientation.w = lmk->getO()->getState()(0);
            lmk_marker.pose.orientation.x = lmk->getO()->getState()(1);
            lmk_marker.pose.orientation.y = lmk->getO()->getState()(2);
            lmk_marker.pose.orientation.z = lmk->getO()->getState()(3);
        }
        // 2D
        else
            lmk_marker.pose.orientation = tf::createQuaternionMsgFromYaw(lmk->getO()->getState()(0));
    }

    // TEXT MARKER ------------------------------------------------------
    lmk_text_marker.text = std::to_string(lmk->id());
    lmk_text_marker.pose.position.x = lmk_marker.pose.position.x;
    lmk_text_marker.pose.position.y = lmk_marker.pose.position.y;
    lmk_text_marker.pose.position.z = lmk_marker.pose.position.z + landmark_text_z_offset_;
}

void WolfRosVisualizer::fillFactorMarker(FactorBaseConstPtr fac,
                                         visualization_msgs::Marker& fac_marker)
{
    geometry_msgs::Point point1, point2;

    // point1 -> frame ------------------------------------------------------
    point1.x = fac->getCapture()->getFrame()->getP()->getState()(0);
    point1.y = fac->getCapture()->getFrame()->getP()->getState()(1);
    if (fac->getCapture()->getFrame()->getP()->getSize() > 2)
        point1.z = fac->getCapture()->getFrame()->getP()->getState()(2);
    else
        point1.z = 0;

    // point2 -> other ------------------------------------------------------
    // FRAME
    if (fac->getFrameOther() != nullptr)
    {
        point2.x = fac->getFrameOther()->getP()->getState()(0);
        point2.y = fac->getFrameOther()->getP()->getState()(1);
        if (fac->getFrameOther()->getP()->getSize() > 2)
            point2.z = fac->getFrameOther()->getP()->getState()(2);
        else
            point2.z = 0;
    }
    // CAPTURE (with Frame)
    else if (fac->getCaptureOther() != nullptr &&
             fac->getCaptureOther()->getFrame() != nullptr)
    {
        point2.x = fac->getCaptureOther()->getFrame()->getP()->getState()(0);
        point2.y = fac->getCaptureOther()->getFrame()->getP()->getState()(1);
        if (fac->getCaptureOther()->getFrame()->getP()->getSize() > 2)
            point2.z = fac->getCaptureOther()->getFrame()->getP()->getState()(2);
        else
            point2.z = 0;
    }
    // FEATURE (with Frame)
    else if (fac->getFeatureOther() != nullptr &&
             fac->getFeatureOther()->getCapture() != nullptr &&
             fac->getFeatureOther()->getCapture()->getFrame() != nullptr)
    {
        point2.x = fac->getFeatureOther()->getCapture()->getFrame()->getP()->getState()(0);
        point2.y = fac->getFeatureOther()->getCapture()->getFrame()->getP()->getState()(1);
        if (fac->getFeatureOther()->getCapture()->getFrame()->getP()->getSize() > 2)
            point2.z = fac->getFeatureOther()->getCapture()->getFrame()->getP()->getState()(2);
        else
            point2.z = 0;
    }
    // LANDMARK
    else if (fac->getLandmarkOther() != nullptr)
    {
        point2.x = fac->getLandmarkOther()->getP()->getState()(0);
        point2.y = fac->getLandmarkOther()->getP()->getState()(1);
        if (fac->getLandmarkOther()->getP()->getSize() > 2)
            point2.z = fac->getLandmarkOther()->getP()->getState()(2);
        else
            point2.z = 0;
    }
    // ABSOLUTE
    else
    {
        point2 = point1;
        point2.z = 20;
    }

    // store points ------------------------------------------------------
    fac_marker.points.push_back(point1);
    fac_marker.points.push_back(point2);

    // colors ------------------------------------------------------
    auto color = (fac->getStatus() == FAC_ACTIVE ? color_active_ : color_inactive_);
    fac_marker.colors.push_back(color);
    fac_marker.colors.push_back(color);
}


void WolfRosVisualizer::fillFrameMarker(FrameBaseConstPtr frm,
                                         visualization_msgs::Marker& frm_marker)
{
    // SHAPE ------------------------------------------------------
    // Position-> SPHERE
    // Pose -> ARROW
    if (frm->getO() != nullptr)
    {
        landmark_marker_.type = visualization_msgs::Marker::ARROW;
        frm_marker.scale.x = frame_length_;
        frm_marker.scale.y = frame_width_;
        frm_marker.scale.z = frame_width_;
    }
    else
    {
        landmark_marker_.type = visualization_msgs::Marker::SPHERE;
        frm_marker.scale.x = frame_width_;
        frm_marker.scale.y = frame_width_;
        frm_marker.scale.z = frame_width_;
    }

    // POSITION & ORIENTATION ------------------------------------------------------
    // position
    frm_marker.pose.position.x = frm->getP()->getState()(0);
    frm_marker.pose.position.y = frm->getP()->getState()(1);
    if (frm->getP()->getSize() > 2)
        frm_marker.pose.position.z = frm->getP()->getState()(2);
    else
        frm_marker.pose.position.z = 0;

    // orientation
    if (frm->getO() != nullptr)
    {
        // 3D
        if (frm->getO()->getSize() > 1)
        {
            frm_marker.pose.orientation.w = frm->getO()->getState()(0);
            frm_marker.pose.orientation.x = frm->getO()->getState()(1);
            frm_marker.pose.orientation.y = frm->getO()->getState()(2);
            frm_marker.pose.orientation.z = frm->getO()->getState()(3);
        }
        // 2D
        else
            frm_marker.pose.orientation = tf::createQuaternionMsgFromYaw(frm->getO()->getState()(0));
    }
}
