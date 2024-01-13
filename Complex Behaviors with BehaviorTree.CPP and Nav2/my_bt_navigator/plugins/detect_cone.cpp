#include "detect_cone.hpp"
#include <iostream>

DetectConeCondition::DetectConeCondition(
    const std::string &condition_name,
    const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf),
      is_cone_detected_(false), callback_received_(false) // Initialize member variables
{
    // Get the topic name from the blackboard
    getInput("detection_topic", detection_topic_);

    // Get the node from the blackboard
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    // Create a callback group
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    // Add the callback group to the executor
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Configure the subscription
    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    // Create the subscriber
    cone_detection_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
        detection_topic_,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&DetectConeCondition::coneDetectionCallback, this, std::placeholders::_1),
        sub_option);
}

BT::NodeStatus DetectConeCondition::tick()
{
    callback_group_executor_.spin_some();

    if (!callback_received_) {
        std::cout << "Waiting for cone detection callback..." << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    if (is_cone_detected_) {
        std::cout << "Cone is detected!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout << "Cone not detected!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

void DetectConeCondition::coneDetectionCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    is_cone_detected_ = msg->data;
    callback_received_ = true;
    std::cout << "Received cone detection message: " << is_cone_detected_ << std::endl;
}




#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<DetectConeCondition>("DetectCone");
}
