//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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

#ifndef NODE_H
#define NODE_H

/**************************
 *   WOLF ROS includes    *
 **************************/
#include "subscriber.h"
#include "publisher.h"

/**************************
 *      WOLF includes     *
 **************************/
#include <core/common/wolf.h>
#include <core/solver/solver_manager.h>
#include <core/problem/problem.h>
#include <core/utils/loader.h>


/**************************
 *     ROS includes     *
 **************************/
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
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


using namespace wolf;

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
        std::vector<std::shared_ptr<Loader>> loaders_;

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

        double node_rate_;

        WolfRosNode();

        virtual ~WolfRosNode(){};

        void solve();
        void solveLoop();

        void print();

        void createProfilingFile();
};

#endif // NODE_H
