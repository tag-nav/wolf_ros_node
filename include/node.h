/**************************
 *      WOLF includes     *
 **************************/
#include <core/common/node_base.h>
#include <core/common/wolf.h>
#include <core/capture/capture_odom_2d.h>
#include <core/ceres_wrapper/solver_ceres.h>
#include <core/sensor/sensor_odom_2d.h>
#include <core/processor/processor_odom_2d.h>
#include <core/problem/problem.h>
#include <core/utils/loader.h>
#include <core/yaml/parser_yaml.h>
#include <core/solver/factory_solver.h>


/**************************
 *     CERES includes     *
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
#include <fstream>
#include <string>

#include "subscriber.h"
#include "publisher.h"

using namespace wolf;
using namespace std;

class WolfRosNode
{
    public:
        //wolf problem
        ProblemPtr problem_ptr_;

        // ROS node handle
        ros::NodeHandle nh_;

        // subscribers
        std::vector<SubscriberPtr> subscribers_;
        // publishers
        std::vector<PublisherPtr> publishers_;


    protected:
        // solver
        SolverManagerPtr solver_;
        ros::Time last_cov_stamp_;

        // profiling
        bool profiling_;
        std::ofstream profiling_file_;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_experiment_;

        // print
        bool print_problem_;
        double print_period_;
        ros::Time last_print_;
        int print_depth_;
        bool print_constr_by_, print_metric_, print_state_blocks_;


      public:
        WolfRosNode();

        virtual ~WolfRosNode(){};

        void solve();
        void solveLoop();

        void print();

        void createProfilingFile();
};
