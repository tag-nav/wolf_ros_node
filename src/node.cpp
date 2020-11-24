#include "node.h"
#include "core/solver/factory_solver.h"
#include "ros/time.h"
#include "tf/transform_datatypes.h"
#include "factory_subscriber.h"
#include "factory_publisher.h"

WolfRosNode::WolfRosNode()
    : nh_(ros::this_node::getName())
    , state_available_(true)
{
    // ROS PARAMS
    std::string yaml_file, plugins_path, subscribers_path;
    nh_.param<std::string>("yaml_file_path", yaml_file, ros::package::getPath("wolf_ros_node")+"/yaml/params_demo.yaml");
    nh_.param<std::string>("plugins_path", plugins_path, "/usr/local/lib/iri-algorithms/");
    nh_.param<std::string>("packages_path", subscribers_path, ros::package::getPath("wolf_ros_node") + "/../../devel/lib/");
    nh_.param<std::string>("map_frame_id",   map_frame_id_,  "map");
    nh_.param<std::string>("odom_frame_id",  odom_frame_id_, "odom");
    nh_.param<std::string>("base_frame_id",  base_frame_id_, "base_footprint");

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

    // TF INIT
    ROS_INFO("Initializing TF...");
    updateTf();
    broadcastTf();

    // PROFILING
    profiling_ = server.getParam<bool>("profiling/enabled");
    if (profiling_)
    {
        auto prof_file = server.getParam<std::string>("profiling/file_name");
        // change ~ with HOME using environment variable
        if (prof_file.at(0) == '~')
            prof_file = std::string(std::getenv("HOME")) + prof_file.substr(1);

        profiling_file_.open (prof_file);
        if (not profiling_file_.is_open())
            ROS_ERROR("Error in opening file %s to store profiling!", prof_file.c_str());
    }

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
        else if (solver_->getVerbosity() != SolverManager::ReportVerbosity::QUIET)
            ROS_WARN("Failed to compute covariances");
    }
}

bool WolfRosNode::updateTf()
{
    ROS_DEBUG("================updateTf==================");

    // get current vehicle pose
    bool result = true;

    VectorComposite current_state = problem_ptr_->getState("PO");
    TimeStamp loc_ts = problem_ptr_->getTimeStamp();

    //Get map2base from Wolf result, and builds base2map pose
    tf::Transform T_map2base;
    if (current_state.count('P') == 0 or
        current_state.count('O') == 0 or
        !loc_ts.ok())
    {
        if (state_available_)
        {
            ROS_WARN("State not available...");
            state_available_ = false; // warning won't be displayed again
        }
        T_map2base.setIdentity();
    }
    else
    {
        if (not state_available_)
        {
            ROS_INFO("State available!");
            state_available_ = true; // warning won't be displayed again
        }
        // 2D
        if (problem_ptr_->getDim() == 2)
        {
            T_map2base = tf::Transform (tf::createQuaternionFromYaw(current_state['O'](0)),
                                        tf::Vector3(current_state['P'](0), current_state['P'](1), 0) );
        }
        // 3D
        else
        {
            T_map2base = tf::Transform (tf::Quaternion(current_state['O'](0), current_state['O'](1), current_state['O'](2), current_state['O'](3)),
                                        tf::Vector3(current_state['P'](0), current_state['P'](1), current_state['P'](2)) );
        }
    }

    //gets T_map2odom_ (odom wrt map), by using tf listener, and assuming an odometry node is broadcasting odom2base
    tf::StampedTransform T_base2odom;
    ros::Time loc_stamp(loc_ts.getSeconds(), loc_ts.getNanoSeconds());
    if ( tfl_.waitForTransform(base_frame_id_, odom_frame_id_, ros::Time(0), ros::Duration(0.2)) )
    {
        // tfl_.lookupTransform(base_frame_id_, odom_frame_id_, loc_stamp, T_base2odom);
        tfl_.lookupTransform(base_frame_id_, odom_frame_id_, ros::Time(0), T_base2odom);
        //std::cout << ros::Time::now().sec << " Odometry: " << T_base2odom.inverse().getOrigin().getX() << " " << T_base2odom.inverse().getOrigin().getY() << " " << T_base2odom.inverse().getRotation().getAngle() << std::endl;
    }
    else
    {
        ROS_WARN("No %s to %s frame received", base_frame_id_.c_str(), odom_frame_id_.c_str());
        T_base2odom.setIdentity();
        T_base2odom.frame_id_ = base_frame_id_;
        T_base2odom.child_frame_id_ = odom_frame_id_;
        T_base2odom.stamp_ = loc_stamp;
        result = false;
    }

    // Broadcast transform ---------------------------------------------------------------------------
    // tf::StampedTransform T_map2odom(T_map2base * T_base2odom, loc_stamp, map_frame_id_, odom_frame_id_);
    // this->T_map2odom = tf::StampedTransform(T_map2base * T_base2odom, loc_stamp, map_frame_id_, odom_frame_id_);
    this->T_map2odom = tf::Transform(T_map2base * T_base2odom);
    //std::cout << "T_map2odom: " << T_map2odom.getOrigin().getX() << " " << T_map2odom.getOrigin().getY() << " " << T_map2odom.getRotation().getAngle() << std::endl;
    return result;
    //T_map2odom.setData(T_map2base * T_base2odom);
    //T_map2odom.stamp_ = loc_stamp;
}
void WolfRosNode::broadcastTf()
{
    auto current_map2odom = tf::StampedTransform(this->T_map2odom, ros::Time::now(), map_frame_id_, odom_frame_id_);
    tfb_.sendTransform(current_map2odom);
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

    // print
    std::cout << profiling_str.str();

    // file
    profiling_file_ << profiling_str.str();
    profiling_file_.close();
}

int main(int argc, char **argv)
{
    std::cout << "\n=========== WOLF ROS WRAPPER MAIN ===========\n\n";

    // Init ROS
    ros::init(argc, argv, ros::this_node::getName());
    // Wolf node
    WolfRosNode wolf_node;

    ros::Rate loopRate(100);
    ros::Time last_viz_time = ros::Time(0);
    ros::Time last_solve_time = ros::Time(0);

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
        // broadcast tf
        wolf_node.updateTf();
        wolf_node.broadcastTf();

        // publish periodically
        for(auto pub : wolf_node.publishers_)
            if (pub->ready())
                pub->publish();

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
