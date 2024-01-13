#ifndef DETECT_CONE_CONDITION_HPP
#define DETECT_CONE_CONDITION_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <mutex>

class DetectConeCondition : public BT::ConditionNode
{
public:
    // Constructor
    DetectConeCondition(const std::string &name,
                        const BT::NodeConfiguration &conf);

    DetectConeCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("detection_topic", "/cone_detection", "Cone topic"),
            BT::InputPort<bool>("is_cone")
        };
    }

private:
    void coneDetectionCallback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cone_detection_subscriber_;
    bool is_cone_detected_;
    bool callback_received_;
    std::string detection_topic_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

#endif // DETECT_CONE_CONDITION_HPP
