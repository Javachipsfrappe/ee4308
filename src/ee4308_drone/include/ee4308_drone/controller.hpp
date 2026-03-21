#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ee4308_drone/core.hpp"

#pragma once

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::drone
{
    class Controller : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
        rclcpp::TimerBase::SharedPtr timer_;

        // other states
        nav_msgs::msg::Odometry odom_;
        nav_msgs::msg::Path plan_;
        bool received_odom_;

        // params
        bool enable_;
        double frequency_;
        double lookahead_distance_;
        double max_xy_vel_;
        double max_z_vel_;
        double yaw_vel_;
        double kp_xy_;
        double kp_z_;

    public:
        explicit Controller(
            const rclcpp::NodeOptions &options,
            const std::string &name);

    private:
        void callbackSubOdom_(const nav_msgs::msg::Odometry msg);

        void callbackSubPlan_(const nav_msgs::msg::Path msg);

        void callbackTimer_();

        void publishCmdVel_(double x_vel, double y_vel, double z_vel, double yaw_vel);
    };
}