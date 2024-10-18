#include "lifecycle_node.hpp"

namespace kinematic_icp_ros {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn KinematicICPOnlineNode::on_configure(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Configuring");
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    InitializePoseAndExtrinsic();
    InitializePublishersAndSubscribers();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, get_node_base_interface());
}
}  // namespace kinematic_icp_ros
