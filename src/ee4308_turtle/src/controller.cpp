#include "ee4308_turtle/controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ee4308::turtle
{
    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        this->node_ = parent.lock();
        this->tf_ = tf;
        this->plugin_name_ = name;

        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_linear_vel", this->desired_linear_vel_, 0.2);
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_lookahead_dist", this->desired_lookahead_dist_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".lookahead_gain", this->lookahead_gain_, 0.5);
        ee4308::initParam(this->node_, this->plugin_name_ + ".min_lookahead_dist", this->min_lookahead_dist_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_lookahead_dist", this->max_lookahead_dist_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_angular_vel", this->max_angular_vel_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_linear_vel", this->max_linear_vel_, 0.22);
        ee4308::initParam(this->node_, this->plugin_name_ + ".xy_goal_thres", this->xy_goal_thres_, 0.05);
    }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &rbt_pose_odom,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)goal_checker;

        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0.0, 0.0);
        }

        geometry_msgs::msg::PoseStamped rbt_pose;
        tf_->transform(rbt_pose_odom, rbt_pose, "map");

        const auto &goal_pose = global_plan_.poses.back();

        auto clamp = [](double v, double lo, double hi) {
            return std::min(std::max(v, lo), hi);
        };
        auto hypot2 = [](double x, double y) { return std::sqrt(x * x + y * y); };

        const double rx = rbt_pose.pose.position.x;
        const double ry = rbt_pose.pose.position.y;
        const double r_yaw = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation);

        const double gx = goal_pose.pose.position.x;
        const double gy = goal_pose.pose.position.y;

        const double dist_to_goal = hypot2(gx - rx, gy - ry);
        if (dist_to_goal <= xy_goal_thres_)
        {
            return writeCmdVel(0.0, 0.0);
        }

        // Find closest point on path
        size_t closest_idx = 0;
        double best_d2 = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < global_plan_.poses.size(); ++i)
        {
            const double px = global_plan_.poses[i].pose.position.x;
            const double py = global_plan_.poses[i].pose.position.y;
            const double dx = px - rx;
            const double dy = py - ry;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2)
            {
                best_d2 = d2;
                closest_idx = i;
            }
        }

        // Vary lookahead with speed
        const double v_now = std::fabs(velocity.linear.x);
        const double Ld_des = clamp(
            desired_lookahead_dist_ + lookahead_gain_ * v_now,
            min_lookahead_dist_, max_lookahead_dist_);

        // Pick lookahead point
        geometry_msgs::msg::PoseStamped lookahead_pose = goal_pose; // fallback
        for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i)
        {
            const double px = global_plan_.poses[i].pose.position.x;
            const double py = global_plan_.poses[i].pose.position.y;
            if (hypot2(px - rx, py - ry) >= Ld_des)
            {
                lookahead_pose = global_plan_.poses[i];
                break;
            }
        }

        // Transform lookahead point into robot frame
        const double lx = lookahead_pose.pose.position.x;
        const double ly = lookahead_pose.pose.position.y;
        const double dx = lx - rx;
        const double dy = ly - ry;

        const double cy = std::cos(r_yaw);
        const double sy = std::sin(r_yaw);

        const double x_r = cy * dx + sy * dy;
        const double y_r = -sy * dx + cy * dy;

        // If lookahead is behind, rotate to face it
        if (x_r <= 1e-4)
        {
            const double w = clamp(std::atan2(y_r, x_r), -max_angular_vel_, max_angular_vel_);
            return writeCmdVel(0.0, w);
        }

        const double Ld = std::max(1e-4, hypot2(x_r, y_r));
        const double curvature = (2.0 * y_r) / (Ld * Ld);

        const double linear_vel = std::min(desired_linear_vel_, max_linear_vel_);
        const double angular_vel = clamp(linear_vel * curvature, -max_angular_vel_, max_angular_vel_);

        return writeCmdVel(linear_vel, angular_vel);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "odom";
        cmd_vel.header.stamp = this->node_->now();
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    void Controller::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }
    void Controller::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }
    void Controller::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { this->global_plan_ = path; }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)