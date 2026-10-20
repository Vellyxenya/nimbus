#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <tf2_ros/transform_broadcaster.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "nimbus/nimbus.hpp"
#include "nimbus/occupancy_grid.hpp"

namespace nimbus
{

/**
 * @brief ROS 2 adapter for sensing, local mapping, and visualisation.
 *
 * It subscribes to game velocity and laser scans, integrates the velocity into
 * an estimated map-frame position, and publishes a local occupancy map, a bird
 * position marker, and the `map -> laser_frame` transform. It publishes the
 * first MPC acceleration command unless the `inhibit_control` parameter is set.
 */
class NimbusRos
{
  public:
    /**
     * @brief Connects all game topic subscriptions and visualisation publishers.
     * @param node ROS node that owns the publishers, subscriptions, and clock.
     */
    explicit NimbusRos(rclcpp::Node::SharedPtr node);

  private:
    /** @brief Advances the dead-reckoned pose to a ROS-clock timestamp. */
    void advancePoseTo(int64_t time_ns);

    /** @brief Stores the latest noise-free game velocity for pose propagation. */
    void velocityCallback(const geometry_msgs::msg::Vector3& msg);

    /** @brief Fuses a scan, then publishes the local map, bird marker, and TF. */
    void laserScanCallback(const sensor_msgs::msg::LaserScan& msg);

    /** @brief Publishes the full-map raw-obstacle distance field as coloured RViz cells.
     */
    void publishDistanceField(const builtin_interfaces::msg::Time& stamp);

    /** @brief Replans and publishes the first MPC action for every laser scan. */
    void planAndPublishTrajectory(const builtin_interfaces::msg::Time& stamp);

    /** @brief Reports the end state and resets the ROS-independent autonomy state. */
    void gameEndedCallback(const std_msgs::msg::Bool& msg);

    /// Node that owns all ROS communication objects.
    rclcpp::Node::SharedPtr node_;
    /// Objective constructed from ROS parameters when the node starts.
    ObjectiveWeights objective_weights_;
    /// ROS-free autonomy code configured with the active objective.
    Nimbus nimbus_;
    /// Forward-looking rolling obstacle map built from laser scans.
    RollingOccupancyGrid map_;

    /// Integrated map-frame x position in metres.
    double position_x_{0.0};
    /// Integrated map-frame y position in metres.
    double position_y_{0.0};
    /// Most recent x velocity in metres per second.
    double velocity_x_{0.0};
    /// Most recent y velocity in metres per second.
    double velocity_y_{0.0};
    /// Timestamp represented by the current integrated pose.
    std::optional<int64_t> last_pose_time_ns_;
    /// Suppresses all acceleration publication when true, for visual-only testing.
    bool inhibit_control_{false};
    /// Publishes MPC acceleration commands on `/nimbus_acc`.
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_acceleration_command_;
    /// Publishes the local occupancy window on `/nimbus/local_map`.
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_local_map_;
    /// Publishes the obstacle-inflated planning layer on `/nimbus/inflated_map`.
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_inflated_map_;
    /// Publishes green-to-red raw-obstacle distances for RViz on
    /// `/nimbus/distance_field`.
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_distance_field_;
    /// Publishes the selected MPC trajectory on `/nimbus/planned_trajectory`.
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_planned_trajectory_;
    /// Publishes the cyan bird marker on `/nimbus/integrated_position`.
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_bird_position_;
    /// Receives `/nimbus_vel`.
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_velocity_;
    /// Receives `/nimbus_laser_scan`.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
    /// Receives `/nimbus_game_ended`.
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_game_ended_;
    /// Publishes the dynamic `map -> laser_frame` transform.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace nimbus
