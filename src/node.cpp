//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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

// wolf ros
#include "factory_subscriber.h"
#include "factory_publisher.h"

// wolf
#include <core/solver/factory_solver.h>
#include <core/yaml/parser_yaml.h>

// ros
#include <node.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>

// std
#include <chrono>

WolfRosNode::WolfRosNode()
    : nh_(ros::this_node::getName())
    , last_print_(ros::Time(0))
{
    // ROS PARAMS
    std::string yaml_file, plugins_path, packages_path;
    nh_.param<std::string>("yaml_file_path",
                           yaml_file,
                           ros::package::getPath("wolf_ros_node") + "/yaml/params_demo.yaml");
    nh_.param<std::string>("plugins_path",
                           plugins_path,
                           "/usr/local/lib/");
    nh_.param<std::string>("packages_path",
                           packages_path,
                           ros::package::getPath("wolf_ros_node") + "/../../devel/lib/");

    // PARAM SERVER CONFIGURATION
    std::cout << "yaml: " << yaml_file << std::endl;
    int found = yaml_file.find_last_of("\\/");

    std::string     yaml_dir    = yaml_file.substr(0, found);
    ParserYaml      parser      = ParserYaml(yaml_file, yaml_dir);
    ParamsServer    server      = ParamsServer(parser.getParams());

    server.addParam("plugins_path", plugins_path);
    server.print();

    // PROBLEM
    ROS_INFO("Creating problem...");
    problem_ptr_ = Problem::autoSetup(server);

    // SOLVER
    ROS_INFO("Creating solver...");
    solver_ = FactorySolver::create("SolverCeres", problem_ptr_, server);

    // ROS
    node_rate_ = server.getParam<double>("problem/node_rate");

    // LOAD PACKAGES (subscribers and publishers)
#if __APPLE__
    std::string lib_extension = ".dylib";
#else
    std::string lib_extension = ".so";
#endif
    
    // PUBLISHERS
    ROS_INFO("Creating publishers...");
    for (auto it : server.getParam<std::vector<std::map<std::string, std::string>>>("ROS publisher"))
    {
        std::string type            = it["type"];
        std::string topic           = it["topic"];
        std::string package         = it["package"];
        std::string name            = type + " - " + topic;
        std::string lib_publisher   = packages_path + "/libpublisher_" + package + lib_extension;

        WOLF_TRACE("Loading publisher " + type + " via " + lib_publisher);
        auto l = std::make_shared<LoaderRaw>(lib_publisher);
        l->load();
        loaders_.push_back(l);

        WOLF_INFO("Pub: ", type, " name: ", name);
        publishers_.push_back(FactoryPublisher::create(type,
                                                       name,
                                                       server,
                                                       problem_ptr_,
                                                       nh_));
    }

    // SUBSCRIBERS
    ROS_INFO("Creating subscribers...");
    for (auto it : server.getParam<std::vector<std::map<std::string, std::string>>>("ROS subscriber"))
    {
        std::string type            = it["type"];
        std::string topic           = it["topic"];
        std::string sensor          = it["sensor_name"];
        std::string package         = it["package"];
        std::string name            = type + " - " + topic;
        std::string lib_subscriber  = packages_path + "/libsubscriber_" + package + lib_extension;

        WOLF_TRACE("Loading subscriber " + type + " via " + lib_subscriber);
        auto l = std::make_shared<LoaderRaw>(lib_subscriber);
        l->load();
        loaders_.push_back(l);

        WOLF_TRACE("From sensor {" + sensor + "} subscribing {" + type + "} to {" + topic + "} topic");
        subscribers_.push_back(FactorySubscriber::create(type,
                                                         name,
                                                         server,
                                                         problem_ptr_->findSensor(sensor),
                                                         nh_));
    }

    // PROFILING
    profiling_ = server.getParam<bool>("debug/profiling");
    if (profiling_)
    {
        auto prof_file = server.getParam<std::string>("debug/profiling_file");
        // change ~ with HOME using environment variable
        if (prof_file.at(0) == '~')
            prof_file = std::string(std::getenv("HOME")) + prof_file.substr(1);

        profiling_file_.open (prof_file);
        if (not profiling_file_.is_open())
            ROS_ERROR("Error in opening file %s to store profiling!", prof_file.c_str());
    }

    //  PRINT
    print_problem_ = server.getParam<bool>("debug/print_problem");
    if(print_problem_)
    {
        print_period_       = server.getParam<double>   ("debug/print_period");
        print_depth_        = server.getParam<int>      ("debug/print_depth");
        print_constr_by_    = server.getParam<bool>     ("debug/print_constr_by");
        print_metric_       = server.getParam<bool>     ("debug/print_metric");
        print_state_blocks_ = server.getParam<bool>     ("debug/print_state_blocks");
        last_print_ = ros::Time::now();
        if (print_depth_ > 4 or print_depth_ < 0)
            throw std::runtime_error("Wrong parameter value, 'debug/print_depth' should be 0, 1, 2, 3 or 4");
    }

    start_experiment_ = std::chrono::high_resolution_clock::now();

    ROS_INFO("Ready!");
}

void WolfRosNode::solve()
{
    if (solver_->getVerbosity() != SolverManager::ReportVerbosity::QUIET)
        ROS_INFO("================ solve ==================");

    std::string report = solver_->solve();

    if (!report.empty())
        std::cout << report << std::endl;

    if (solver_->getParams()->compute_cov and (ros::Time::now() - last_cov_stamp_).toSec() > solver_->getCovPeriod())
    {
        auto start = std::chrono::high_resolution_clock::now();
        if (solver_->computeCovariances())
        {
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
            last_cov_stamp_ = ros::Time::now();
            if (solver_->getVerbosity() != SolverManager::ReportVerbosity::QUIET)
                ROS_INFO("Covariances computed successfully! It took %li microseconds", duration.count());
        }
        else
        {
            // will try again after 10% of cov period
            last_cov_stamp_ = last_cov_stamp_+ ros::Duration(0.1*solver_->getCovPeriod());
            if (solver_->getVerbosity() != SolverManager::ReportVerbosity::QUIET)
                ROS_WARN("Failed to compute covariances");
        }
    }
}

void WolfRosNode::solveLoop()
{
    auto awake_time = std::chrono::system_clock::now();
    auto period = std::chrono::duration<int,std::milli>((int)(solver_->getPeriod()*1e3+1)); // 1ms added to allow pausing if rosbag paused if period==0
    WOLF_DEBUG("Started solver loop");

    while (ros::ok())
    {
        solve();

        if(ros::isShuttingDown())
            break;

        // relax to fit output rate
        awake_time += period;
        std::this_thread::sleep_until(awake_time);
    }
    WOLF_DEBUG("Solver loop finished");
}

void WolfRosNode::print()
{
    if(print_problem_ and
       (ros::Time::now() - last_print_).toSec() >= print_period_)
    {
        problem_ptr_->print(print_depth_, print_constr_by_, print_metric_, print_state_blocks_);
        last_print_ = ros::Time::now();
    }
}

void WolfRosNode::createProfilingFile()
{
    if (not profiling_)
        return;

    std::stringstream profiling_str;
    profiling_str << "========== WOLF PROFILING ==========\n";
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_experiment_);
    profiling_str << "Experiment total time: " << 1e-6 * duration.count() << " s" << std::endl;

    // solver
    profiling_str << "\nSOLVER -----------------------------\n";
    solver_->printProfiling(profiling_str);

    // processors
    profiling_str << "\nPROCESSORS -------------------------\n";
    for (auto sensor : problem_ptr_->getHardware()->getSensorList())
        for (auto proc : sensor->getProcessorList())
             proc->printProfiling(profiling_str);

    // publishers
    profiling_str << "\nPUBLISHERS -------------------------\n";
    for (auto pub : publishers_)
        pub->printProfiling(profiling_str);

    profiling_str << "\n";

    // print
    std::cout << profiling_str.str();

    // file
    profiling_file_ << profiling_str.str();
    profiling_file_.close();
}

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, ros::this_node::getName());

    // Wolf node
    WolfRosNode wolf_node;

    ros::Rate loopRate(wolf_node.node_rate_);

    // periodic stuff
    ros::Time last_check = ros::Time::now();

    // Solver thread
    std::thread solver_thread(&WolfRosNode::solveLoop, &wolf_node);
    // set priority
    struct sched_param Priority_Param; //struct to set priority
    int priority = 99;
    Priority_Param.sched_priority = priority;
    int policy=SCHED_FIFO;
    pthread_setschedparam(solver_thread.native_handle(), SCHED_FIFO, &Priority_Param);

    // Init publishers threads
    for(auto pub : wolf_node.publishers_)
    {
        WOLF_INFO("Running publisher ", pub->getName());
        pub->run();
    }

    while (ros::ok())
    {
        // check that subscribers received data (every second)
        if ((ros::Time::now() - last_check).toSec() > 1)
        {
            for (auto sub : wolf_node.subscribers_)
                WOLF_WARN_COND(sub->secondsSinceLastCallback() > 5, sub->getName(), " has not received any callback for ", sub->secondsSinceLastCallback(), "s.");
            last_check = ros::Time::now();
        }

        // print periodically
        wolf_node.print();

        // execute pending callbacks
        ros::spinOnce();

        // relax to fit output rate
        loopRate.sleep();
    }

    // Stop solver thread
    WOLF_DEBUG("Node is shutting down outside loop... waiting for the thread to stop...");
    solver_thread.join();
    WOLF_DEBUG("thread stopped.");

    // Stop publishers threads
    for(auto pub : wolf_node.publishers_)
        pub->stop();

    // PROFILING ========================================
    wolf_node.createProfilingFile();

    return 0;
}
