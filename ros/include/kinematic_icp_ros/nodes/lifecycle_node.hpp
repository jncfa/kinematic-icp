// MIT License

// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp/node_options.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sophus/se3.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>

// STL
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace kinematic_icp_ros {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class KinematicICPOnlineNode : public rclcpp_lifecycle::LifecycleNode {
public:
    KinematicICPOnlineNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : rclcpp_lifecycle::LifecycleNode("online_node", "kinematic_icp", options){};

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    /*
     * @brief Lifecycle activate
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    /*
     * @brief Lifecycle deactivate
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    /*
     * @brief Lifecycle cleanup
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    /*
     * @brief Lifecycle shutdown
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    void InitializePoseAndExtrinsic();
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    void InitializePublishersAndSubscribers();
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        kpoints_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
        voxel_grid_pub_;
    rclcpp::Subscription<sensor_msgs::PointCloud2>::ConstSharedPtr pointcloud_sub_;

    /// SetPose service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_pose_srv_;

    /// Internal messages, models the current state that will be published by this node.
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TransformStamped tf_msg_;

    /// Global/map coordinate frame.
    std::string lidar_odom_frame_{"odom_lidar"};
    std::string wheel_odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
    // TF frame initialization flag
    bool initialize_odom_node{false};
};
}  // namespace kinematic_icp_ros
