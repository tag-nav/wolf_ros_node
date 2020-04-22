/**************************
 *      WOLF includes     *
 **************************/
#include <core/common/node_base.h>
#include <core/common/wolf.h>
#include <core/capture/capture_odom_2d.h>
#include <core/sensor/sensor_odom_2d.h>
#include <core/processor/processor_odom_2d.h>
#include <core/problem/problem.h>
#include <core/utils/loader.h>
#include <core/yaml/parser_yaml.h>
#include <core/solver/factory_solver.h>


/**************************
 *     CERES includes     *
 **************************/
#include "core/ceres_wrapper/ceres_manager.h"
//#include "glog/logging.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include "tf/LinearMath/Transform.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <iomanip>
#include <queue>
#include <memory>

#include "subscriber.h"
#include "visualizer.h"
#include "publisher.h"
//#include "wolf_ros_scan_visualizer.h"

using namespace wolf;
using namespace std;

class WolfRosNode
{
    public:
        //wolf problem
        ProblemPtr problem_ptr_;

        // ROS node handle
        ros::NodeHandle nh_;

        double solver_period_;
        double viz_period_;
        double publisher_period_;

        // visualizer
        std::shared_ptr<Visualizer> viz_;

        // subscribers
        std::vector<SubscriberPtr> subscribers_;
        // publishers
        std::vector<PublisherPtr> publishers_;

    protected:
        // solver
        SolverManagerPtr solver_manager_ptr_;
        SolverManager::ReportVerbosity solver_verbose_;

        // transforms
        tf::TransformBroadcaster tfb_;
        tf::TransformListener    tfl_;
        std::string base_frame_id_, map_frame_id_, odom_frame_id_;
        tf::Transform T_map2odom;


      public:
        WolfRosNode();

        virtual ~WolfRosNode(){};

        void solve();

        void broadcastTf();

        void visualize();

        bool updateTf();
};