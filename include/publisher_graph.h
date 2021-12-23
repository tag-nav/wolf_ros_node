//--------LICENSE_START--------
//
// Copyright (C) 2020,2021 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
#ifndef PUBLISHER_GRAPH_H
#define PUBLISHER_GRAPH_H

/**************************
 *      WOLF includes     *
 **************************/
#include "core/problem/problem.h"
#include "publisher.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace wolf
{

class PublisherGraph: public Publisher
{
    public:
        PublisherGraph(const std::string& _unique_name,
                      const ParamsServer& _server,
                      const ProblemPtr _problem);
        WOLF_PUBLISHER_CREATE(PublisherGraph);

        virtual ~PublisherGraph(){};

        void initialize(ros::NodeHandle &nh, const std::string& topic) override;

        void publishDerived() override;

    protected:

        void publishLandmarks();
        void publishFactors();
        void publishTrajectory();

        virtual void fillLandmarkMarkers(LandmarkBaseConstPtr        lmk,
                                         visualization_msgs::Marker& lmk_marker,
                                         visualization_msgs::Marker& lmk_text_marker);
        virtual void fillFactorMarker(FactorBaseConstPtr          fac,
                                      visualization_msgs::Marker& fac_marker,
                                      visualization_msgs::Marker& fac_text_marker);
        virtual void fillFrameMarker(FrameBaseConstPtr           frm,
                                     visualization_msgs::Marker& frm_marker,
                                     visualization_msgs::Marker& frm_text_marker);

        std::string factorString(FactorBaseConstPtr fac) const;

        // publishers
        ros::Publisher landmarks_publisher_;
        ros::Publisher trajectory_publisher_;
        ros::Publisher factors_publisher_;

        // Marker arrayss
        visualization_msgs::MarkerArray landmarks_marker_array_;
        visualization_msgs::MarkerArray trajectory_marker_array_;
        visualization_msgs::MarkerArray factors_marker_array_;

        // Markers
        visualization_msgs::Marker landmark_marker_, landmark_text_marker_;
        visualization_msgs::Marker frame_marker_, frame_text_marker_;
        visualization_msgs::Marker factor_marker_, factor_text_marker_;

        // Options
        std::string map_frame_id_;
        bool        viz_overlapped_factors_, viz_inactive_factors_;
        double      viz_scale_, text_scale_, factors_width_, factors_absolute_height_, landmark_text_z_offset_, landmark_width_, landmark_length_, frame_width_, frame_length_, frame_vel_scale_;
        std_msgs::ColorRGBA frame_color_, factor_abs_color_, factor_motion_color_, factor_loop_color_, factor_lmk_color_, factor_geom_color_, factor_other_color_;

        // auxiliar variables
        unsigned int landmark_max_hits_;
        double       viz_period_;
        ros::Time    last_markers_publish_;
        std::set<std::string> factors_drawn_;


};

WOLF_REGISTER_PUBLISHER(PublisherGraph)

}

#endif
