#include <cmath>
#include "ee4308_turtle/controller.hpp"

namespace ee4308::turtle
{
    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        this->node_ = parent.lock(); 
        this->tf_ = tf;
        this->plugin_name_ = name;

        // initialize parameters
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_linear_vel", this->desired_linear_vel_, 0.2);
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_lookahead_dist", this->desired_lookahead_dist_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_angular_vel", this->max_angular_vel_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_linear_vel", this->max_linear_vel_, 0.22);
        ee4308::initParam(this->node_, this->plugin_name_ + ".xy_goal_thres", this->xy_goal_thres_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".yaw_goal_thres", this->yaw_goal_thres_, 0.25);

        // initialize new parameters for Regulated Pure Pursuit
        ee4308::initParam(this->node_, this->plugin_name_ + ".curvature_thres", this->curvature_thres_, 0.1);
        ee4308::initParam(this->node_, this->plugin_name_ + ".proximity_thres", this->proximity_thres_, 0.1);
        ee4308::initParam(this->node_, this->plugin_name_ + ".lookahead_gain", this->lookahead_gain_, 1.5);
        ee4308::initParam(this->node_, this->plugin_name_ + ".angular_kp", this->angular_kp_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".angular_ki", this->angular_ki_, 0.1);

        // initialize state variables
        this->prev_linear_vel_ = this->desired_linear_vel_;
        this->adj_lookahead_dist_ = this->desired_lookahead_dist_;
        this->angular_integral_ = 0.0;
        this->last_angular_time_ = this->node_->now();

        // initialize topics (uncommented for obstacle heuristic)
        this->sub_scan_ = this->node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubScan_, this, std::placeholders::_1));
    }

    void Controller::callbackSubScan_(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        this->scan_ranges_ = msg->ranges;
    }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &rbt_pose_odom,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     
        (void)goal_checker; 

        // 2. If no global path Then
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0); // 3. Return (0,0)
        }
        // 4. End If

        geometry_msgs::msg::PoseStamped rbt_pose;
        tf_->transform(rbt_pose_odom, rbt_pose, "map");
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();

        // 5. If the robot is close to the goal Then
        double dist_to_goal = ee4308::getDistance(rbt_pose.pose.position, goal_pose.pose.position);
        if (dist_to_goal < xy_goal_thres_)
        {
            double goal_yaw = ee4308::getYawFromQuaternion(goal_pose.pose.orientation);
            double rbt_yaw = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation);
            double yaw_diff = goal_yaw - rbt_yaw;

            // Normalize angle strictly between -PI and PI
            while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;

            // 6. If the robot's heading far from the goal orientation Then
            if (std::abs(yaw_diff) > yaw_goal_thres_)
            {
                // 7. Return (0, w0)
                double w0 = std::copysign(max_angular_vel_ * 0.5, yaw_diff);
                return writeCmdVel(0, w0);
            }
            // 8. End If
            
            // 9. Return (0, 0)
            return writeCmdVel(0, 0); 
        }
        // 10. End If

        // 11. Find the point along the path that is closest to the robot.
        double rx = rbt_pose.pose.position.x;
        double ry = rbt_pose.pose.position.y;
        size_t closest_idx = 0;
        double min_dist_sq = 1e30;
        for (size_t i = 0; i < global_plan_.poses.size(); ++i)
        {
            double px = global_plan_.poses[i].pose.position.x;
            double py = global_plan_.poses[i].pose.position.y;
            double dx = px - rx, dy = py - ry;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }

        // 12. From the closest point, find the lookahead point. (Using adjusted lookahead)
        size_t lookahead_idx = closest_idx;
        for (size_t j = closest_idx; j < global_plan_.poses.size(); ++j)
        {
            double d = ee4308::getDistance(rbt_pose.pose.position, global_plan_.poses[j].pose.position);
            if (d >= adj_lookahead_dist_)
            {
                lookahead_idx = j;
                break;
            }
            lookahead_idx = j; 
        }
        const auto &lookahead_pose = global_plan_.poses[lookahead_idx];

        // 13. Transform the lookahead point into the robot frame to get (x', y').
        double yaw = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation);
        double lx = lookahead_pose.pose.position.x;
        double ly = lookahead_pose.pose.position.y;
        double dx = lx - rx, dy = ly - ry;
        double xp = dx * std::cos(yaw) + dy * std::sin(yaw);
        double yp = -dx * std::sin(yaw) + dy * std::cos(yaw);

        // Angle to lookahead point from robot frame (forward = 0, positive = left)
        double angle_to_lookahead = std::atan2(yp, xp);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 200,
                            "angle_to_lookahead: %.3f rad (%.1f deg)", angle_to_lookahead, angle_to_lookahead * 180.0 / M_PI);

        double L_sq = xp * xp + yp * yp;
        if (L_sq < ee4308::THRES * ee4308::THRES) {
            angular_integral_ = 0.0;
            return writeCmdVel(0, 0);
        }

        // Get distance to closest obstacle (do) from laser scan
        double d_o = 1e30;
        for (float r : scan_ranges_) {
            if (std::isfinite(r) && r > 0.05) { 
                if (r < d_o) d_o = r;
            }
        }
        // PI controller: angular_vel = Kp * angle_to_lookahead + Ki * integral(angle)
        rclcpp::Time now = node_->now();
        double dt = (now - last_angular_time_).seconds();
        if (dt > 0.0 && dt < 1.0)
            angular_integral_ += angle_to_lookahead * dt;
        last_angular_time_ = now;
        double angular_vel = angular_kp_ * angle_to_lookahead + angular_ki_ * angular_integral_;
        angular_vel = angular_vel * (d_o / proximity_thres_);
        // Use curvature only for linear vel heuristic (curvature_thres, c_abs)
        // double c_abs = 2.0 * std::abs(yp) / L_sq;

        // 16. Calculate the curvature heuristic.
        // double v_prime = prev_linear_vel_; 
        // double v_c;
        // if (curvature_thres_ < c_abs) {
        //     v_c = desired_linear_vel_ * (curvature_thres_ / c_abs);
        // } else {
        //     // Tend back towards desired speed when not curvature-limited
        //     v_c = desired_linear_vel_; 
        // }

        // 17. Calculate the obstacle heuristic.
        double v;
        if (d_o < proximity_thres_) {
            v = desired_linear_vel_ * (d_o / proximity_thres_);
        } else {
            v = desired_linear_vel_;
        }
        // linear_vel from heuristics; angular_vel from PI controller (already computed above)
        if (abs(angle_to_lookahead) > M_PI / 4) {
            v = 0;
        }
        double linear_vel = v;
        // 18. Vary the lookahead. (Used in the next time-step)
        adj_lookahead_dist_ = linear_vel * lookahead_gain_;
        
        // Safety bounds for lookahead distance
        if (adj_lookahead_dist_ < 0.1) adj_lookahead_dist_ = 0.1;
        if (adj_lookahead_dist_ > 2.0) adj_lookahead_dist_ = 2.0;

        // 19 & 20. Constrain w and v to within the largest allowable speeds.
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        linear_vel = std::clamp(linear_vel, 0.0, max_linear_vel_);

        // Update previous linear velocity for next iteration
        // prev_linear_vel_ = linear_vel;

        // 21. Return (v, w)
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

    void Controller::setPlan(const nav_msgs::msg::Path &path)
    {
        this->global_plan_ = path;
        angular_integral_ = 0.0;
        last_angular_time_ = node_->now();
    }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)
