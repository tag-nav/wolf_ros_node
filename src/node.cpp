#include "node.h"
#include "ros/time.h"
#include "core/solver/factory_solver.h"
#include "tf/transform_datatypes.h"
#include "factory_subscriber.h"
#include "factory_publisher.h"

WolfRosNode::WolfRosNode()
    : nh_(ros::this_node::getName())
{
    // ROS PARAMS
    std::string yaml_file, plugins_path, subscribers_path;
    nh_.param<std::string>("yaml_file_path", yaml_file, ros::package::getPath("wolf_ros_node")+"/yaml/params_demo.yaml");
    nh_.param<std::string>("plugins_path", plugins_path, "/usr/local/lib/iri-algorithms/");
    nh_.param<std::string>("packages_path", subscribers_path, ros::package::getPath("wolf_ros_node") + "/../../devel/lib/");

    // PARAM SERVER CONFIGURATION
    std::cout <<"yaml: " << yaml_file << std::endl;
    int found = yaml_file.find_last_of("\\/");
    std::string yaml_dir = yaml_file.substr(0, found);
    ParserYaml parser = ParserYaml(yaml_file, yaml_dir);
    ParamsServer server = ParamsServer(parser.getParams());

    server.addParam("plugins_path", plugins_path);
    server.addParam("packages_path", subscribers_path);

    server.print();

    // PROBLEM
    ROS_INFO("Creating problem...");
    problem_ptr_ = Problem::autoSetup(server);

    // SOLVER
    ROS_INFO("Creating solver...");
    solver_ = FactorySolver::create("SolverCeres", problem_ptr_, server);
    // covariance
    compute_cov_ = server.getParam<bool>("solver/compute_cov");
    if (compute_cov_)
    {
        cov_enum_   = (SolverManager::CovarianceBlocksToBeComputed)server.getParam<int>("solver/cov_enum");
        cov_period_ = server.getParam<double>("solver/cov_period");
    }

    // ROS SUBSCRIBERS
    ROS_INFO("Creating subscribers...");
    for (auto it : server.getParam<std::vector<std::map<std::string, std::string>>>("ROS subscriber"))
    {
        std::string subscriber = it["type"];
        std::string topic      = it["topic"];
        std::string sensor     = it["sensor_name"];
        WOLF_TRACE("From sensor {" + sensor + "} subscribing {" + subscriber + "} to {" + topic + "} topic");
        subscribers_.push_back(FactorySubscriber::create(subscriber, subscriber+" - "+topic, server, problem_ptr_->getSensor(sensor), nh_));
    }

    // ROS PUBLISHERS
    ROS_INFO("Creating publishers...");
    for (auto it : server.getParam<std::vector<std::map<std::string, std::string>>>("ROS publisher"))
    {
        WOLF_INFO("Pub: ", it["type"]);
        publishers_.push_back(FactoryPublisher::create(it["type"], it["type"]+" - "+it["topic"], server, problem_ptr_, nh_));
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
    //  DEBUG
    print_problem_ = server.getParam<bool>("debug/print_problem");
    if(print_problem_)
    {
        print_period_ = server.getParam<double>("debug/print_period");
        last_print_ = ros::Time::now();
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
    {
        std::cout << report << std::endl;
        //problem_ptr_->print(4,1,1,1);
    }

    if (compute_cov_ and (ros::Time::now() - last_cov_stamp_).toSec() > cov_period_)
    {
        auto start = std::chrono::high_resolution_clock::now();
        if (solver_->computeCovariances(cov_enum_))
        {
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
            last_cov_stamp_ = ros::Time::now();
            if (solver_->getVerbosity() != SolverManager::ReportVerbosity::QUIET)
                ROS_INFO("Covariances computed successfully! It took %li microseconds", duration.count());
        }
        else
        {
            // will try again after 10% of cov period
            last_cov_stamp_ = last_cov_stamp_+ ros::Duration(0.1*cov_period_);
            if (solver_->getVerbosity() != SolverManager::ReportVerbosity::QUIET)
                ROS_WARN("Failed to compute covariances");
        }
    }
}

void WolfRosNode::solveLoop()
{
    ros::Rate solverRate(1/(solver_->getPeriod()+1e-9)); // 1ns added to allow pausing if rosbag paused if period==0
    WOLF_DEBUG("Started solver loop");

    while (ros::ok())
    {
        solve();

        if(ros::isShuttingDown())
            break;

        // relax to fit output rate
        solverRate.sleep();
    }
    WOLF_DEBUG("Solver loop finished");
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

    ros::Rate loopRate(100);

    // periodic stuff
    ros::Time last_check = ros::Time::now();
    wolf_node.last_print_ = ros::Time(0);

    // Solver thread
    std::thread solver_thread(&WolfRosNode::solveLoop, &wolf_node);
    // set priority
    struct sched_param Priority_Param; //struct to set priority
    int priority = 99;
    Priority_Param.sched_priority = priority;
    int policy=SCHED_FIFO;
    pthread_setschedparam(solver_thread.native_handle(), SCHED_FIFO, &Priority_Param);

    while (ros::ok())
    {
        // publish periodically
        for(auto pub : wolf_node.publishers_)
            if (pub->ready())
                pub->publish();

        // check subscribers (every second)
        if ((ros::Time::now() - last_check).toSec() > 1)
        {
            for (auto sub : wolf_node.subscribers_)
                WOLF_WARN_COND(sub->secondsSinceLastCallback() > 5, sub->getName(), " has not received any callback for ", sub->secondsSinceLastCallback(), "s.");
            last_check = ros::Time::now();
        }

        // print periodically
        if(wolf_node.print_problem_ and
           (ros::Time::now() - wolf_node.last_print_).toSec() >= wolf_node.print_period_)
        {
            wolf_node.problem_ptr_->print(4,1,1,1);
            wolf_node.last_print_ = ros::Time::now();
        }

        // execute pending callbacks
        ros::spinOnce();

        // relax to fit output rate
        loopRate.sleep();
    }
    WOLF_DEBUG("Node is shutting down outside loop... waiting for the thread to stop...");
    solver_thread.join();
    WOLF_DEBUG("thread stopped.");

    // PROFILING ========================================
    wolf_node.createProfilingFile();

    return 0;
}
