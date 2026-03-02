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

        // initialize states / variables
        this->node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        this->tf_ = tf;
        this->plugin_name_ = name;

        // initialize parameters
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_linear_vel", this->desired_linear_vel_, 0.2);
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_lookahead_dist", this->desired_lookahead_dist_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_angular_vel", this->max_angular_vel_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_linear_vel", this->max_linear_vel_, 0.22);
        ee4308::initParam(this->node_, this->plugin_name_ + ".xy_goal_thres", this->xy_goal_thres_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".yaw_goal_thres", this->yaw_goal_thres_, 0.25);
        ee4308::initParam(this->node_, this->plugin_name_ + ".prox_dist", this->prox_dist_, 0.6);   // d_prox
        ee4308::initParam(this->node_, this->plugin_name_ + ".prox_fov_deg", this->prox_fov_deg_, 60.0); // +/- FOV

        // initialize topics
        this->sub_scan_ = this->node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubScan_, this, std::placeholders::_1));
    }

    void Controller::callbackSubScan_(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        this->scan_ranges_ = msg->ranges;
        angle_min_ = msg->angle_min;
        angle_increment_ = msg->angle_increment;
    }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &rbt_pose_odom,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        // get rbt's pose in map frame (DO NOT DELETE --> need the next two lines for rbt_pose)
        geometry_msgs::msg::PoseStamped rbt_pose;
        tf_->transform(rbt_pose_odom, rbt_pose, "map");

        // get goal pose (contains the "clicked" goal rotation and position)
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();
        
        //some helpful lambda functions
        auto clamp = [](double v, double lo, double hi) {
            // limits a value within a range
            return std::min(std::max(v, lo), hi);
        };

        auto wrap_pi = [](double a) {
            while (a > M_PI) a -= 2.0 * M_PI;
            while (a < -M_PI) a += 2.0 * M_PI;
            return a;
        };

        auto hypot2 = [](double x, double y) { return std::sqrt(x * x + y * y); };

        const double rx = rbt_pose.pose.position.x;
        const double ry = rbt_pose.pose.position.y;
        const double r_yaw = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation);

        const double gx = goal_pose.pose.position.x;
        const double gy = goal_pose.pose.position.y;
        const double g_yaw = ee4308::getYawFromQuaternion(goal_pose.pose.orientation);

        const double dist_to_goal = hypot2(gx - rx, gy - ry);
        const double yaw_err = wrap_pi(g_yaw - r_yaw);

        // Stop condition: close to within position and yaw threshold
        if (dist_to_goal <= xy_goal_thres_ && std::fabs(yaw_err) <= yaw_goal_thres_)
        {
            return writeCmdVel(0.0, 0.0);
        }

        // Goal position but yaw isn't done yet, will rotate in place
        if (dist_to_goal <= xy_goal_thres_)
        {
            const double k_yaw = 1.5; // P controller
            const double w = clamp(k_yaw * yaw_err, -max_angular_vel_, max_angular_vel_);
            return writeCmdVel(0.0, w);
        }

        // Pure pursuit: Find closest point on the path
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

        // Pure pursuit: Choose a lookahead point
        const double Ld_des = std::max(0.05, desired_lookahead_dist_);
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

        // Pure pursuit: Transform lookahead point into robot frame
        const double lx = lookahead_pose.pose.position.x;
        const double ly = lookahead_pose.pose.position.y;
        const double dx = lx - rx;
        const double dy = ly - ry;

        const double cy = std::cos(r_yaw);
        const double sy = std::sin(r_yaw);

        const double x_r = cy * dx + sy * dy;
        const double y_r = -sy * dx + cy * dy;

        // Not a pure pursuit: Handle lookahead point is behind
        if (x_r <= 1e-4)
        {
            const double w = clamp(1.0 * std::atan2(y_r, x_r), -max_angular_vel_, max_angular_vel_);
            return writeCmdVel(0.0, w);
        }

        const double Ld = std::max(1e-4, hypot2(x_r, y_r));

        const double curvature = (2.0 * y_r) / (Ld * Ld);

        // Base speed (no curvature-based scaling)
        double linear_vel = std::min(desired_linear_vel_, max_linear_vel_);

        // ---------- Proximity heuristic ----------
        if (!scan_ranges_.empty())
        {
            // Find minimum obstacle distance in a front cone
            // Assumes scan is in base frame and angle_min..angle_max are valid
            // If your LaserScan meta isn't stored, this is the main "risk" (see notes below).
            
            // NOTE: If you don't store angle_min / increment in callback, you must use msg directly.
            // So recommended: store angle_min_, angle_inc_ too in callback.
        }

        // Simple robust version: store angle_min_ and angle_increment_ in callbackSubScan_
        // then do:

        double d_o = std::numeric_limits<double>::infinity();

        if (!scan_ranges_.empty())
        {
            const double half_fov = (prox_fov_deg_ * M_PI / 180.0) * 0.5;

            for (size_t i = 0; i < scan_ranges_.size(); ++i)
            {
                const double ang = angle_min_ + static_cast<double>(i) * angle_increment_;
                if (std::fabs(ang) > half_fov) continue;

                const double r = static_cast<double>(scan_ranges_[i]);
                if (!std::isfinite(r)) continue;
                d_o = std::min(d_o, r);
            }

            // Proximity scaling: v = v * (d_o / d_prox) if d_o < d_prox
            if (std::isfinite(d_o) && d_o < prox_dist_)
            {
                linear_vel *= (d_o / prox_dist_);
            }
        }

        // clamp (purely safety)
        linear_vel = clamp(linear_vel, 0.0, max_linear_vel_);

        // Pure pursuit angular command (unchanged structure)
        double angular_vel = linear_vel * curvature;
        angular_vel = clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

        double heading_err = std::atan2(y_r, x_r); // in robot frame

        if (std::abs(heading_err) > 1.0) { // ~57 degrees
            linear_vel = 0.0;
            angular_vel = clamp(1.0 * heading_err, -max_angular_vel_, max_angular_vel_);
        }
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

    // ======================================== DO NOT TOUCH =================================

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