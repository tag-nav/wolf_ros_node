/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

/**************************
 *      WOLF includes     *
 **************************/
#include "core/common/wolf.h"
#include "core/problem/problem.h"

using namespace wolf;

class WolfRosVisualizer
{
    public:
        bool publish_markers_;

        WolfRosVisualizer();

        void initialize(ros::NodeHandle& nh);

        virtual ~WolfRosVisualizer(){};

        void visualize(const ProblemPtr problem);

    private:

        void publishLandmarks(const ProblemPtr problem);
        void publishFactors(const ProblemPtr problem);
        void publishTrajectory(const ProblemPtr problem);

        void fillLandmarkMarkers(LandmarkBaseConstPtr lmk,
                                 visualization_msgs::Marker& lmk_marker,
                                 visualization_msgs::Marker& lmk_text_marker);
        void fillFactorMarker(FactorBaseConstPtr fac,
                              visualization_msgs::Marker& fac_marker);
        void fillFrameMarker(FrameBaseConstPtr frm,
                             visualization_msgs::Marker& frm_marker);


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
        visualization_msgs::Marker factor_marker_;

        // Options
        std::string map_frame_id_;
        bool viz_factors_, viz_landmarks_, viz_trajectory_;
        double factors_width_, landmark_text_z_offset_, landmark_width_, landmark_length_, frame_width_, frame_length_;
        std_msgs::ColorRGBA color_active_, color_inactive_;

        // auxiliar variables
        unsigned int landmark_max_hits_;
        double viz_period_;
        ros::Time last_markers_publish_;
};
