#ifndef TRAJECTORY_MANAGER_HPP_
#define TRAJECTORY_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/empty.hpp"
// #include "trajectory_msgs/srv/set_string.hpp"
#include <fstream>
#include <mutex>

class TrajectoryManager {
public:
    TrajectoryManager(const std::shared_ptr<rclcpp::Node>& node_);

    // save the path as center line in global coordinate
    void savePathCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void startRecordingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void stopRecordingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);

    // // extract a lookahead path in frenet coordinate given the current position (odometry)
    // void extractLookaheadPath(const nav_msgs::msg::Odometry& odom, nav_msgs::msg::Path& lookahead_path);

    // // save and load a path file
    // void savePathToFile(const nav_msgs::msg::Path& path);
    // nav_msgs::msg::Path loadPathFromFile();

    // Service callback functions
    

private:
    // ROS node and publishers/subscribers
    rclcpp::Node::SharedPtr node;
    
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_recording_srv_, stop_recording_srv_, save_path_srv_;
    


    // Path recording variables
    nav_msgs::msg::Path recorded_path_;
    bool is_recording_;

    // Mutex for thread safety
    std::mutex mutex_;

};

#endif  // TRAJECTORY_MANAGER_HPP_
