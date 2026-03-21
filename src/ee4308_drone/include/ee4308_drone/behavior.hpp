#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ee4308_drone/core.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#pragma once

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::drone
{

    class Behavior : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_est_pose_; // ground truth pose and twist
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_turtle_plan_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_turtle_stop_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_;

        rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr cli_plan_;
        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::Odometry odom_;
        nav_msgs::msg::Path turtle_plan_;
        std::shared_future<std::shared_ptr<nav_msgs::srv::GetPlan::Response>> request_plan_future_;
        bool plan_requested_;
        bool turtle_stop_;

        // parameters
        std::string topic_turtle_stop_;
        std::string topic_turtle_plan_;
        std::string frame_id_map_;
        double initial_x_;
        double initial_y_;
        double initial_z_;
        double reached_thres_;
        double cruise_height_;
        double frequency_;

        // state "enums"
        static constexpr int BEGIN = 0;           // initial state before takeoff.
        static constexpr int TAKEOFF = 1;         // takeoff from initial position.
        static constexpr int TURTLE_POSITION = 2; // fly to turtle's position.
        static constexpr int TURTLE_WAYPOINT = 3; // fly to turtle's current waypoint.
        static constexpr int INITIAL = 4;         // fly to above initial position.
        static constexpr int LANDING = 5;         // land from above initial position.
        static constexpr int END = 6;             // on the ground at initial position after landing.
        int state_;
        double waypoint_x_;
        double waypoint_y_;
        double waypoint_z_;

    public:
        explicit Behavior(
            const rclcpp::NodeOptions &options,
            const std::string &name);

    private:
        void callbackSubOdom_(nav_msgs::msg::Odometry::SharedPtr msg);

        void callbackSubTurtleStop_(std_msgs::msg::Empty::SharedPtr msg);

        void callbackSubTurtlePlan_(nav_msgs::msg::Path::SharedPtr msg);

        void callbackTimer_();

        void transition_(int new_state);

        void setWaypoint_(double waypoint_x, double waypoint_y, double waypoint_z);

        bool reachedWaypoint_();

        void requestPlan_(double drone_x, double drone_y, double drone_z,
                          double waypoint_x, double waypoint_y, double waypoint_z);

        void callbackCliReceivePlan_(const rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future);
    };
}