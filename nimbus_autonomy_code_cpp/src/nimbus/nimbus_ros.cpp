#include "nimbus/nimbus_ros.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include "nimbus/planning/motion_model.hpp"

namespace nimbus
{

namespace
{

inline constexpr uint32_t kQueueSize = 5U;

ObjectiveWeights declareObjectiveWeightParameters(const rclcpp::Node::SharedPtr& node)
{
    ObjectiveWeights weights;
    weights.progress =
            node->declare_parameter<double>("objective.progress", weights.progress);
    weights.clearance =
            node->declare_parameter<double>("objective.clearance", weights.clearance);
    weights.vertical_velocity = node->declare_parameter<double>(
            "objective.vertical_velocity", weights.vertical_velocity);
    weights.control_effort = node->declare_parameter<double>("objective.control_effort",
                                                             weights.control_effort);
    weights.control_rate = node->declare_parameter<double>("objective.control_rate",
                                                           weights.control_rate);
    weights.collision =
            node->declare_parameter<double>("objective.collision", weights.collision);
    weights.early_collision = node->declare_parameter<double>("objective.early_collision",
                                                              weights.early_collision);
    weights.minimum_clearance = node->declare_parameter<double>(
            "objective.minimum_clearance", weights.minimum_clearance);
    return weights;
}

}  // namespace

const char* collisionReasonName(CollisionReason reason)
{
    switch (reason)
    {
        case CollisionReason::kNone:
            return "none";
        case CollisionReason::kInflatedObstacle:
            return "inflated obstacle";
        case CollisionReason::kMapBoundary:
            return "map boundary";
    }
    return "unknown";
}

NimbusRos::NimbusRos(rclcpp::Node::SharedPtr node)
    : node_(node),
      objective_weights_(declareObjectiveWeightParameters(node_)),
      nimbus_(objective_weights_)
{
    RCLCPP_INFO(
            node_->get_logger(),
            "Objective weights: progress=%.3f, clearance=%.3f, vertical_velocity=%.3f, "
            "control_effort=%.3f, control_rate=%.3f, collision=%.3f, "
            "early_collision=%.3f, minimum_clearance=%.3f",
            objective_weights_.progress, objective_weights_.clearance,
            objective_weights_.vertical_velocity, objective_weights_.control_effort,
            objective_weights_.control_rate, objective_weights_.collision,
            objective_weights_.early_collision, objective_weights_.minimum_clearance);
    inhibit_control_ = node_->declare_parameter<bool>("inhibit_control", false);
    pub_acceleration_command_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
            "/nimbus_acc", kQueueSize);
    pub_local_map_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/nimbus/local_map", kQueueSize);
    pub_inflated_map_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/nimbus/inflated_map", kQueueSize);
    pub_distance_field_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/nimbus/distance_field", kQueueSize);
    pub_planned_trajectory_ = node_->create_publisher<nav_msgs::msg::Path>(
            "/nimbus/planned_trajectory", kQueueSize);
    pub_bird_position_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "/nimbus/integrated_position", kQueueSize);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    sub_velocity_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
            "/nimbus_vel", kQueueSize,
            std::bind(&NimbusRos::velocityCallback, this, std::placeholders::_1));
    sub_laser_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/nimbus_laser_scan", kQueueSize,
            std::bind(&NimbusRos::laserScanCallback, this, std::placeholders::_1));
    sub_game_ended_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/nimbus_game_ended", kQueueSize,
            std::bind(&NimbusRos::gameEndedCallback, this, std::placeholders::_1));
}

void NimbusRos::advancePoseTo(int64_t time_ns)
{
    if (!last_pose_time_ns_.has_value())
    {
        last_pose_time_ns_ = time_ns;
        return;
    }

    const double dt = (time_ns - *last_pose_time_ns_) * 1e-9;
    if (dt <= 0.0)
    {
        return;
    }
    position_x_ += velocity_x_ * dt;
    position_y_ += velocity_y_ * dt;
    last_pose_time_ns_ = time_ns;
}

void NimbusRos::velocityCallback(const geometry_msgs::msg::Vector3& msg)
{
    advancePoseTo(node_->now().nanoseconds());
    velocity_x_ = msg.x;
    velocity_y_ = msg.y;
}

void NimbusRos::laserScanCallback(const sensor_msgs::msg::LaserScan& msg)
{
    const int64_t scan_time_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
    advancePoseTo(scan_time_ns > 0 ? scan_time_ns : node_->now().nanoseconds());

    std::vector<double> angles;
    std::vector<bool> hits;
    angles.reserve(msg.ranges.size());
    hits.reserve(msg.ranges.size());
    for (size_t index = 0; index < msg.ranges.size(); ++index)
    {
        angles.push_back(msg.angle_min + index * msg.angle_increment);
        hits.push_back(index < msg.intensities.size() && msg.intensities[index] > 0.5F);
    }
    map_.updateScan(position_x_, position_y_, angles, msg.ranges, hits);

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = msg.header.stamp;
    grid.header.frame_id = "map";
    grid.info.resolution = map_.resolution();
    grid.info.width = map_.width();
    grid.info.height = map_.height();
    grid.info.origin.position.x = map_.originX();
    grid.info.origin.position.y = map_.originY();
    grid.info.origin.orientation.w = 1.0;
    grid.data = map_.occupancy();
    pub_local_map_->publish(grid);

    auto inflated_grid = grid;
    inflated_grid.data = map_.inflatedOccupancy();
    pub_inflated_map_->publish(inflated_grid);

    publishDistanceField(msg.header.stamp);

    visualization_msgs::msg::Marker bird_marker;
    bird_marker.header.stamp = msg.header.stamp;
    bird_marker.header.frame_id = "map";
    bird_marker.ns = "nimbus";
    bird_marker.id = 0;
    bird_marker.type = visualization_msgs::msg::Marker::SPHERE;
    bird_marker.action = visualization_msgs::msg::Marker::ADD;
    bird_marker.pose.position.x = position_x_;
    bird_marker.pose.position.y = position_y_;
    bird_marker.pose.position.z = 0.1;
    bird_marker.pose.orientation.w = 1.0;
    bird_marker.scale.x = 0.22;
    bird_marker.scale.y = 0.22;
    bird_marker.scale.z = 0.22;
    bird_marker.color.r = 0.1F;
    bird_marker.color.g = 0.8F;
    bird_marker.color.b = 1.0F;
    bird_marker.color.a = 1.0F;
    pub_bird_position_->publish(bird_marker);

    planAndPublishTrajectory(msg.header.stamp);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg.header.stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id =
            msg.header.frame_id.empty() ? "laser_frame" : msg.header.frame_id;
    transform.transform.translation.x = position_x_;
    transform.transform.translation.y = position_y_;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(transform);

    const auto hit_count = std::count(hits.begin(), hits.end(), true);
    RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Mapping: pose=(%.2f, %.2f) m, %zu rays, %zu hits, map origin=(%.2f, %.2f) m",
            position_x_, position_y_, msg.ranges.size(), hit_count, map_.originX(),
            map_.originY());
}

void NimbusRos::publishDistanceField(const builtin_interfaces::msg::Time& stamp)
{
    // Scale against the largest finite distance currently in the window, rather
    // than clipping beyond a fixed range, so the complete field remains visible.
    double maximum_distance_m = 0.0;
    bool has_finite_distance = false;
    for (int y = 0; y < static_cast<int>(map_.height()); ++y)
    {
        for (int x = 0; x < static_cast<int>(map_.width()); ++x)
        {
            const double distance_m = map_.rawObstacleDistanceAt(x, y);
            if (std::isfinite(distance_m))
            {
                has_finite_distance = true;
                maximum_distance_m = std::max(maximum_distance_m, distance_m);
            }
        }
    }

    visualization_msgs::msg::Marker distance_field;
    distance_field.header.stamp = stamp;
    distance_field.header.frame_id = "map";
    distance_field.ns = "distance_field";
    distance_field.id = 0;
    distance_field.type = visualization_msgs::msg::Marker::CUBE_LIST;
    distance_field.action = visualization_msgs::msg::Marker::ADD;
    distance_field.pose.orientation.w = 1.0;
    distance_field.scale.x = map_.resolution();
    distance_field.scale.y = map_.resolution();
    distance_field.scale.z = 0.005;
    distance_field.points.reserve(static_cast<size_t>(map_.width()) * map_.height());
    distance_field.colors.reserve(distance_field.points.capacity());

    for (int y = 0; y < static_cast<int>(map_.height()); ++y)
    {
        for (int x = 0; x < static_cast<int>(map_.width()); ++x)
        {
            const double distance_m = map_.rawObstacleDistanceAt(x, y);
            double normalized_distance = 0.0;
            if (!has_finite_distance)
            {
                normalized_distance = 1.0;
            }
            else if (maximum_distance_m > 0.0 && std::isfinite(distance_m))
            {
                normalized_distance =
                        std::clamp(distance_m / maximum_distance_m, 0.0, 1.0);
            }

            geometry_msgs::msg::Point point;
            point.x = map_.originX() + (x + 0.5) * map_.resolution();
            point.y = map_.originY() + (y + 0.5) * map_.resolution();
            point.z = 0.01;
            distance_field.points.push_back(point);

            std_msgs::msg::ColorRGBA color;
            color.r = static_cast<float>(1.0 - normalized_distance);
            color.g = static_cast<float>(normalized_distance);
            color.a = 0.20F;
            distance_field.colors.push_back(color);
        }
    }
    pub_distance_field_->publish(distance_field);
}

void NimbusRos::planAndPublishTrajectory(const builtin_interfaces::msg::Time& stamp)
{
    const MotionState current_state{position_x_, position_y_, velocity_x_, velocity_y_};
    const auto planning_started = std::chrono::steady_clock::now();
    const auto trajectory = nimbus_.plan(current_state, map_);
    const auto planning_duration = std::chrono::steady_clock::now() - planning_started;
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "map";
    if (!trajectory.collided)
    {
        for (size_t index = 0; index < trajectory.x.size(); ++index)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = trajectory.x[index];
            pose.pose.position.y = trajectory.y[index];
            pose.pose.position.z = 0.05;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
    }
    pub_planned_trajectory_->publish(path);

    const bool executable = !trajectory.controls.empty() &&
                            (!trajectory.collided || trajectory.is_emergency_fallback ||
                             trajectory.is_initial_state_recovery);
    const AccelerationCommand first_control =
            executable ? trajectory.controls.front() : AccelerationCommand{};
    RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "CEM MPC: cost=%.2f, first acceleration=(%.2f, %.2f) m/s^2, control=%s, "
            "mode=%s, planning=%.1f ms",
            trajectory.cost, first_control.ax, first_control.ay,
            !inhibit_control_ && executable ? "enabled" : "inhibited",
            trajectory.is_initial_state_recovery ? "initial-state recovery" :
            trajectory.is_emergency_fallback     ? "emergency brake" :
                                                   "optimised",
            std::chrono::duration<double, std::milli>(planning_duration).count());
    if (trajectory.planning_time_exhausted || planning_duration >= kMpcPlanningBudget)
    {
        RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "CEM MPC reached its %lld ms planning budget; sampling stopped after "
                "%.1f ms",
                static_cast<long long>(kMpcPlanningBudget.count()),
                std::chrono::duration<double, std::milli>(planning_duration).count());
    }
    if (trajectory.collided)
    {
        RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                trajectory.is_initial_state_recovery ?
                        "CEM MPC starts in inflated occupancy: executing least-cost "
                        "recovery; first collision is %s at step %zu of %zu" :
                        "No feasible CEM trajectory: fallback first collides with %s at "
                        "step %zu of %zu",
                collisionReasonName(trajectory.collision_reason),
                trajectory.first_collision_step, trajectory.controls.size());
    }

    if (!inhibit_control_ && executable)
    {
        const auto control = MotionModel::limitControlForVelocity(
                {position_x_, position_y_, velocity_x_, velocity_y_}, first_control);
        geometry_msgs::msg::Vector3 acceleration;
        acceleration.x = control.ax;
        acceleration.y = control.ay;
        pub_acceleration_command_->publish(acceleration);
    }
}

void NimbusRos::gameEndedCallback(const std_msgs::msg::Bool& msg)
{
    if (msg.data)
    {
        RCLCPP_INFO(node_->get_logger(), "Crash detected at pose=(%.2f, %.2f) m.",
                    position_x_, position_y_);
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "End of countdown at pose=(%.2f, %.2f) m.",
                    position_x_, position_y_);
    }

    nimbus_ = Nimbus(objective_weights_);
}

}  // namespace nimbus
