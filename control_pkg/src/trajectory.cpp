// include necessary headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <thread>
#include <mutex>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "trajectory.h"

#include <cmath>
TrajectoryManager::TrajectoryManager(const std::shared_ptr<rclcpp::Node>& node_) : node(node_){
    // create publishers and subscribers
    is_recording_ = false;
    start_recording_srv_ = node->create_service<std_srvs::srv::Empty>("path_record_init", std::bind(&TrajectoryManager::startRecordingCallback, this, std::placeholders::_1, std::placeholders::_2));
    stop_recording_srv_ = node->create_service<std_srvs::srv::Empty>("path_record_stop", std::bind(&TrajectoryManager::stopRecordingCallback, this, std::placeholders::_1, std::placeholders::_2));
    save_path_srv_ = node->create_service<std_srvs::srv::Empty>("path_save", std::bind(&TrajectoryManager::savePathCallback, this, std::placeholders::_1, std::placeholders::_2));


}


void TrajectoryManager::startRecordingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    is_recording_ = true;
    RCLCPP_INFO(node->get_logger(), "Path Recording Init");
}

void TrajectoryManager::stopRecordingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    is_recording_ = false;
    RCLCPP_INFO(node->get_logger(), "Path Recording Stop");
}



// save the path as center line in global coordinate
void TrajectoryManager::savePathCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    RCLCPP_INFO(node->get_logger(), "Path Saved at");
    // if (recorded_path_.poses.size() > 0) {
    //     std::string filename = req->data;
    //     rosbag::Bag bag;
    //     bag.open(filename, rosbag::bagmode::Write);
    //     bag.write("path", ros::Time::now(), recorded_path_);
    //     bag.close();
    //     res->message = "Recorded path saved to file: " + filename;
    //     res->success = true;
    // } else {
    //     res->message = "No recorded path to save";
    //     res->success = false;
    // }
}


// // extract a lookahead path in frenet coordinate given the current position (odometry)
// oid extractLookaheadPath(const nav_msgs::msg::Odometry& odom, nav_msgs::msg::Path& lookahead_path) {
//     // find the closest point on the path to the current position
//     double closest_distance = std::numeric_limits<double>::max();
//     double closest_s = 0.0;
//     double closest_d = 0.0;
//     for (const auto& pose : path_.poses) {
//         double dx = pose.pose.position.x - odom.pose.pose.position.x;
//         double dy = pose.pose.position.y - odom.pose.pose.position.y;
//         double distance = std::sqrt(dx*dx + dy*dy);
//         if (distance < closest_distance) {
//             closest_distance = distance;
//             closest_s = ...; // TODO: compute the s-coordinate of the closest point in frenet space
//             closest_d = ...; // TODO: compute the d-coordinate of the closest point in frenet space
//         }
//     }

//     // compute the lookahead point in frenet space
//     double lookahead_distance = 10.0; // 10 meters lookahead
//     double lookahead_s = closest_s + lookahead_distance;
//     double lookahead_d = ...; // TODO: compute the d-coordinate of the lookahead point in frenet space

//     // convert the lookahead point back to global coordinates
//     nav_msgs::msg::Path lookahead_path_frenet = convertFrenetToPath({lookahead_s}, {lookahead_d});
//     lookahead_path = lookahead_path_frenet;
// }


// save and load a path file
    // save the path to a file
    // void savePathToFile(const nav_msgs::msg::Path& path) {
    //     std::ofstream file("path.txt");
    //     for (const auto& pose : path.poses) {
    //         file << pose.pose.position.x << "," << pose.pose.position.y << std::endl;
    //     }
    //     file.close();
    // }

//  // load the path from a file
//     nav_msgs::msg::Path loadPathFromFile() {
//         nav_msgs::msg::Path path;
//         std::ifstream file("path.txt");
//         if (!file.is_open()) {
//             return path;
//         }
//         std::string line;
//         while (std::getline(file, line)) {
//             std::istringstream iss(line);
//             std::string x_str, y_str;
//             std::getline(iss, x_str, ',');
//             std::getline(iss, y_str, ',');
//             geometry_msgs::msg::PoseStamped pose;
//             pose.header.frame_id = "map";
//             pose.header.stamp = rclcpp::Time::now();
//             pose.pose.position.x = std::stod(x_str);
//             pose.pose.position.y = std::stod(y_str);
//             path.poses.push_back(pose);
//         }
//         file.close();
//         return path;
//     }



