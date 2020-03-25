#include "wolf_node.h"
#include "core/solver/solver_factory.h"
#include "ros/time.h"
#include "tf/transform_datatypes.h"
#include "subscriber_factory.h"
#include "visualizer_factory.h"
#include "publisher_factory.h"
#include <fstream>
#include <iostream>
#include <string>

WolfRosNode::WolfRosNode() : nh_(ros::this_node::getName())
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
    ParserYAML parser = ParserYAML(yaml_file, yaml_dir);
    ParamsServer server = ParamsServer(parser.getParams());

    server.addParam("plugins_path", plugins_path);
    server.addParam("packages_path", subscribers_path);

    server.print();

    while(not ros::Time::isValid()) sleep(1);
    server.addParam("problem/prior/timestamp", std::to_string(ros::Time::now().sec) + "." + std::to_string(ros::Time::now().nsec));
    // std::cout << "Prior timestamp " << server.getParam<std::string>("problem/prior/timestamp") << "\n";

    // PROBLEM
    problem_ptr_ = Problem::autoSetup(server);

    // SOLVER
    solver_manager_ptr_ = std::static_pointer_cast<CeresManager>(SolverFactory::get().create("CERES", problem_ptr_, server));
    int solver_verbose_int;
    solver_period_ = server.getParam<int>("solver/period");
    solver_verbose_int = server.getParam<int>("solver/verbose");
    solver_verbose_ = static_cast<SolverManager::ReportVerbosity>(solver_verbose_int);

    // ROS SUBSCRIBERS
    for (auto it : server.getParam<std::vector<std::map<std::string, std::string>>>("ROS subscriber"))
    {
        std::string subscriber = it["type"];
        std::string topic      = it["topic"];
        std::string sensor     = it["sensor_name"];
        WOLF_TRACE("From sensor {" + sensor + "} subscribing {" + subscriber + "} to {" + topic + "} topic");
        auto subscriber_wrapper = SubscriberFactory::get().create(subscriber, topic, server, problem_ptr_->getSensor(sensor));
        subscribers_.push_back(subscriber_wrapper);
        subscribers_.back()->initSubscriber(nh_, topic);
    }

    // ROS VISUALIZER
    auto visualizer = server.getParam<std::string>("visualizer/type");
    viz_ = VisualizerFactory::get().create(visualizer);
    viz_->initialize(nh_);
    viz_period_ = server.getParam<int>("visualizer/period");

    // ROS PUBLISHERS
    try
    {
        for (auto it : server.getParam<std::vector<std::map<std::string, std::string>>>("ROS publisher"))
        {
            std::string pub = it["type"];
            WOLF_INFO("Pub: ", pub);
            auto publisher = PublisherFactory::get().create(pub);
            publisher->period_ = converter<double>::convert(it["period"]);
            publishers_.push_back(publisher);
            publishers_.back()->initialize(nh_,it["topic"]);
        }
    }
    catch (MissingValueException& e)
    {
        WOLF_WARN(e.what());
        WOLF_WARN("No publishers found...");
    }

    // TF INIT
    updateTf();
    broadcastTf();
}

void WolfRosNode::solve()
{
    ROS_INFO("================ solve ==================");
    std::string report = solver_manager_ptr_->solve(solver_verbose_);
    if (!report.empty())
        std::cout << report << std::endl;
}

void WolfRosNode::visualize()
{
    ROS_INFO("================ visualize ==================");
    auto start = std::chrono::high_resolution_clock::now();
    viz_->visualize(problem_ptr_);
    auto stop     = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //std::cout << "Visualize took " << duration.count() << " microseconds" << std::endl;
}

bool WolfRosNode::updateTf()
{
    ROS_DEBUG("================updateTf==================");

    // get current vehicle pose
    ros::Time loc_stamp;
    TimeStamp loc_ts;
    Eigen::VectorXd current_pose;
    bool result = true;
    problem_ptr_->getCurrentStateAndStamp(current_pose, loc_ts);

    loc_stamp.nsec = loc_ts.getNanoSeconds();
    loc_stamp.sec = loc_ts.getSeconds();
    // loc_stamp = ros::Time::now();

    //Get map2base from Wolf result, and builds base2map pose
    tf::Transform T_map2base;
    if (problem_ptr_->getDim() == 2)
    {
        T_map2base = tf::Transform (tf::createQuaternionFromYaw(current_pose(2)),
                                    tf::Vector3(current_pose(0), current_pose(1), 0) );
        //T_map2base.setOrigin( tf::Vector3(current_pose(0), current_pose(1), 0) );
        //T_map2base.setRotation( tf::createQuaternionFromYaw(current_pose(2)) );
    }
    else
    {
        T_map2base = tf::Transform (tf::Quaternion(current_pose(3), current_pose(4), current_pose(5), current_pose(6)),
                                    tf::Vector3(current_pose(0), current_pose(1), current_pose(2)) );
    }

    std::cout << "Current pose: " << current_pose.transpose() << std::endl;

    //gets T_map2odom_ (odom wrt map), by using tf listener, and assuming an odometry node is broadcasting odom2base
    tf::StampedTransform T_base2odom;
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

int main(int argc, char **argv)
{
    std::cout << "\n=========== WOLF ROS WRAPPER MAIN ===========\n\n";

    // Init ROS
    ros::init(argc, argv, ros::this_node::getName());
    // Wolf node
    WolfRosNode wolf_node;

    ros::Rate loopRate(20);
    ros::Time last_viz_time = ros::Time(0);
    ros::Time last_solve_time = ros::Time(0);

    while (ros::ok())
    {
        // solve periodically
        if ((ros::Time::now() - last_solve_time).toSec() >= wolf_node.solver_period_)
        {
            wolf_node.solve();
            last_solve_time = ros::Time::now();
        }

        // broadcast tf
        wolf_node.updateTf();
        wolf_node.broadcastTf();

        // visualize periodically
        if ((ros::Time::now() - last_viz_time).toSec() >= wolf_node.viz_period_)
        {
            wolf_node.visualize();
            last_viz_time = ros::Time::now();
        }

        // publish periodically
        for(auto pub : wolf_node.publishers_)
            if ((ros::Time::now() - pub->last_publish_time_).toSec() >= pub->period_)
            {
                pub->publish(wolf_node.problem_ptr_);
                pub->last_publish_time_ = ros::Time::now();
            }

        // execute pending callbacks
        ros::spinOnce();

        // relax to fit output rate
        loopRate.sleep();
    }
    // file.close();
    return 0;
}
