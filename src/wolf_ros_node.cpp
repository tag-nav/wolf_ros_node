#include "wolf_ros_node.h"
#include "ros/time.h"
#include "tf/transform_datatypes.h"
#include <fstream>
#include <iostream>
#include <string>

WolfRosNode::WolfRosNode() : nh_(ros::this_node::getName())
{
    std::string yaml_file, plugins_path, subscribers_path;
    nh_.param<std::string>("yaml_file_path", yaml_file, ros::package::getPath("wolf_ros_node")+"/yaml/params_demo.yaml");
    nh_.param<std::string>("plugins_path", plugins_path, "/usr/local/lib/iri-algorithms/");
    nh_.param<std::string>("packages_path", subscribers_path, ros::package::getPath("wolf_ros_node") + "/../../devel/lib/");

    int found = yaml_file.find_last_of("\\/");
    std::cout << yaml_file.substr(0,found) << " " << yaml_file.substr(found+1);
    std::string yaml_dir = yaml_file.substr(0, found);
    ParserYAML parser = ParserYAML(yaml_file, yaml_dir);
    ParamsServer server = ParamsServer(parser.getParams());
    server.print();
    server.addParam("plugins_path", plugins_path);
    server.addParam("packages_path", subscribers_path);

    problem_ptr_ = Problem::autoSetup(server);

    ceres::Solver::Options ceres_options;
    ceres_manager_ptr_ = std::make_shared<CeresManager>(problem_ptr_, ceres_options);

    for (auto it : server.getParam<std::vector<std::map<std::string, std::string>>>("ROS subscriber managers"))
        {
            std::string subscriber = it["type"];
            std::string topic      = it["topic"];
            std::string sensor     = it["sensor_name"];
            WOLF_TRACE("From sensor {" + sensor + "} subscribing {" + subscriber + "} to {" + topic + "} topic")
                auto subscriber_wrapper = SubscriberFactory::get().create(subscriber, topic, server, problem_ptr_->getSensor(sensor));
            subscribers_.push_back(subscriber_wrapper);
            subscribers_.back()->initSubscriber(nh_, topic);
        }

    // TODO: integrate visualizers into YAML config. (We need to figure out how to have general visualizers first)
    std::vector<std::string> visualizers;
    visualizers.push_back("WolfRosScanVisualizer");
    for(auto const& visualizer: visualizers){
        wolf_viz_ = VisualizerFactory::get().create(visualizer);
        wolf_viz_->initialize(nh_);
    }

    nh_.param<std::string>(  "map_frame_id",   map_frame_id_,  "map");
    nh_.param<std::string>(  "odom_frame_id",  odom_frame_id_, "odom");
    nh_.param<std::string>(  "base_frame_id",  base_frame_id_, "base_footprint");

    broadcastTf();
}

void WolfRosNode::solve()
{
    ROS_INFO("================ solve ==================");
    std::string report = ceres_manager_ptr_->solve(SolverManager::ReportVerbosity::BRIEF);
    std::cout << report << std::endl;
}

void WolfRosNode::visualize()
{
    ROS_INFO("================ visualize ==================");
    wolf_viz_->visualize(problem_ptr_);
}

void WolfRosNode::updateTf()
{
    ROS_INFO("================updateTf==================");

    // get current vehicle pose
    ros::Time loc_stamp;
    TimeStamp loc_ts;
    Eigen::VectorXd current_pose;
    problem_ptr_->getCurrentStateAndStamp(current_pose, loc_ts);

//    loc_stamp.nsec = loc_ts.getNanoSeconds();
//    loc_stamp.sec = loc_ts.getSeconds();
    loc_stamp = ros::Time::now();

    //Get map2base from Wolf result, and builds base2map pose
    tf::Transform T_map2base(tf::createQuaternionFromYaw((double) current_pose(2)),
                             tf::Vector3((double) current_pose(0), (double) current_pose(1), 0) );
    //T_map2base.setOrigin( tf::Vector3((double) current_pose(0), (double) current_pose(1), 0) );
    //T_map2base.setRotation( tf::createQuaternionFromYaw((double) current_pose(2)) );

    std::cout << "Current pose: " << current_pose.transpose() << std::endl;

    //gets T_map2odom_ (odom wrt map), by using tf listener, and assuming an odometry node is broadcasting odom2base
    tf::StampedTransform T_base2odom;
    if ( tfl_.waitForTransform(base_frame_id_, odom_frame_id_, loc_stamp, ros::Duration(0.2)) )
    {
        tfl_.lookupTransform(base_frame_id_, odom_frame_id_, loc_stamp, T_base2odom);
        std::cout << "Odometry: " << T_base2odom.inverse().getOrigin().getX() << " " << T_base2odom.inverse().getOrigin().getY() << " " << T_base2odom.inverse().getRotation().getAngle() << std::endl;
    }
    else
    {
        ROS_WARN("No odom to base frame received");
        T_base2odom.setIdentity();
        T_base2odom.frame_id_ = base_frame_id_;
        T_base2odom.child_frame_id_ = odom_frame_id_;
        T_base2odom.stamp_ = loc_stamp;
    }

    // Broadcast transform ---------------------------------------------------------------------------
    // tf::StampedTransform T_map2odom(T_map2base * T_base2odom, loc_stamp, map_frame_id_, odom_frame_id_);
    this->T_map2odom = tf::StampedTransform(T_map2base * T_base2odom, loc_stamp, map_frame_id_, odom_frame_id_);
    this->T_map2odom = tf::Transform(T_map2base * T_base2odom);
    std::cout << "T_map2odom: " << T_map2odom.getOrigin().getX() << " " << T_map2odom.getOrigin().getY() << " " << T_map2odom.getRotation().getAngle() << std::endl;
    //T_map2odom.setData(T_map2base * T_base2odom);
    //T_map2odom.stamp_ = loc_stamp;
}
void WolfRosNode::broadcastTf()
{
    auto current_map2odom = tf::StampedTransform(this->T_map2odom, ros::Time::now(), map_frame_id_, odom_frame_id_);
    tfb_.sendTransform(current_map2odom);
}

int main(int argc, char **argv) {
    std::cout << "\n=========== WOLF ROS WRAPPER MAIN ===========\n\n";


    // Init ROS
    ros::init(argc, argv, ros::this_node::getName());
    // Wolf node
    WolfRosNode wolf_node;
    int visualize_interval;
    wolf_node.nh_.param<int>("visualize_interval", visualize_interval, 1);
    ros::Rate r(1);

    ros::Rate loopRate(20);
    int n_iterations_solve(10);
    int iteration(0);
    ros::Time last_time = ros::Time(0);
    int last_id = -1;
    std::ofstream file;
    // file.open("/home/jcasals/wolf_debug.out");

    while (ros::ok()) {
        // solve every n iterations
        // if (iteration++ >= n_iterations_solve)
        int current = wolf_node.problem_ptr_->getLastKeyFrame()->id();
        if (current != last_id)
            {
                // file.open("/home/jcasals/random/debug/wolf_debug" + std::to_string(current) + "-" + std::to_string(last_id) + "-before.out");
                // file << "ROSTIME " << ros::Time::now();
                // file << wolf_node.problem_ptr_->printToString();
                // file.close();

                // solve
                wolf_node.solve();

                // file.open("/home/jcasals/random/debug/wolf_debug" + std::to_string(current) + "-" + std::to_string(last_id) + "-after.out");
                // file << "ROSTIME " << ros::Time::now();
                // file << wolf_node.problem_ptr_->printToString();
                // file.close();
                // update tf
                wolf_node.updateTf();
                last_id = current;
            }
        // broadcast tf
        wolf_node.broadcastTf();
        // visualize
        auto Dt = ros::Time::now() - last_time;
        if (Dt.toSec() >= visualize_interval) {
            last_time = ros::Time::now();
            wolf_node.visualize();
            // iteration = 1;
        }
    // execute pending callbacks
    ros::spinOnce();

    // relax to fit output rate
    loopRate.sleep();
    }
    // file.close();
    return 0;
}
