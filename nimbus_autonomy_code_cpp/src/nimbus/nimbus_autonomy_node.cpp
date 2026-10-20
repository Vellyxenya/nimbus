#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nimbus/nimbus_ros.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("nimbus_autonomy");

    // The adapter owns all ROS subscriptions and must outlive `rclcpp::spin`.
    [[maybe_unused]] nimbus::NimbusRos nimbus_ros(node);
    RCLCPP_INFO(node->get_logger(), "Nimbus autonomy node ready.");

    rclcpp::spin(node);

    return 0;
}
