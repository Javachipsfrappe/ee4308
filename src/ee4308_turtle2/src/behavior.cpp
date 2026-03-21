#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ee4308_turtle2/core.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::turtle2
{
    /**
     * The Behavior ROS Node that maintains subscribers and publishers for the Behavior class.
     */
    class Behavior : public rclcpp::Node
    {
    private:
        // parameters
        std::vector<double> waypoints_;
        std::string frame_id_turtle_;
        std::string frame_id_map_;
        double frequency_; // affects the plan rate.
        double reached_thres_;

        // topics, services, timers
        rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr cli_plan_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_stop_;
        std::shared_future<std::shared_ptr<nav_msgs::srv::GetPlan::Response>> request_plan_future_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // other states
        size_t waypoint_idx_;
        bool plan_requested_;

    public:
        explicit Behavior(const rclcpp::NodeOptions &options, const std::string &name = "behavior")
            : Node(name, options)
        {
            // states
            waypoint_idx_ = 0;
            plan_requested_ = false;

            // parameters
            this->frequency_ = getParameter<double>(this, "frequency", 2.0).as_double();
            this->reached_thres_ = getParameter<double>(this, "reached_thres", 0.1).as_double();
            this->waypoints_ = getParameter<std::vector<double>>(this, "waypoints", {}).as_double_array();
            this->frame_id_turtle_ = getParameter<std::string>(this, "frame_id_turtle_", "turtle/base_link").as_string();
            this->frame_id_map_ = getParameter<std::string>(this, "frame_id_map_", "map").as_string();

            // topics
            this->pub_stop_ = this->create_publisher<std_msgs::msg::Empty>(
                "stop", rclcpp::ServicesQoS());

            // tf2
            this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // services
            this->cli_plan_ = this->create_client<nav_msgs::srv::GetPlan>("get_plan");

            // Initialize main loop
            this->timer_ = this->create_timer(1s / this->frequency_, std::bind(&Behavior::callbackTimer_, this));
        }

    private:
        void callbackTimer_()
        {
            // all waypoints processed or no waypoints at all.
            if (waypoint_idx_ >= waypoints_.size())
            {
                this->pub_stop_->publish(std_msgs::msg::Empty());
                timer_ = nullptr; // remove itself.
                RCLCPP_INFO(this->get_logger(), "No more waypoints. %s message published.", this->pub_stop_->get_topic_name());
                return; // reached end of path.
            }

            // get tf2
            geometry_msgs::msg::TransformStamped tf_turtle;
            try
            {
                tf_turtle = this->tf_buffer_->lookupTransform(
                    this->frame_id_map_, this->frame_id_turtle_,
                    tf2::TimePointZero);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Cannot get tf for turtle: %s", ex.what());
                return;
            }

            // get the current waypoint.
            const double &waypoint_x = waypoints_[waypoint_idx_];
            const double &waypoint_y = waypoints_[waypoint_idx_ + 1];

            double rbt_x = tf_turtle.transform.translation.x;
            double rbt_y = tf_turtle.transform.translation.y;

            // if reached the waypoint, request for another waypoint
            double distance = std::hypot(waypoint_x - rbt_x, waypoint_y - rbt_y);
            if (distance < reached_thres_)
            {
                waypoint_idx_ += 2;
                // RCLCPP_INFO_STREAM(this->get_logger(),
                //                    "WAYPOINT " << (waypoint_idx_ / 2) // starts from 1.
                //                                << " (" << waypoint_x << "," << waypoint_y << ") REACHED!");
                this->callbackTimer_(); // recurse with new waypoint_idx_.
                return;
            }

            // request a plan
            if (plan_requested_)
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "No request made as there is no response yet from previous request.");
                return;
            }
            plan_requested_ = true;

            geometry_msgs::msg::PoseStamped waypoint_pose;
            waypoint_pose.pose.position.x = waypoint_x;
            waypoint_pose.pose.position.y = waypoint_y;
            waypoint_pose.header.frame_id = this->frame_id_map_;
            waypoint_pose.header.stamp = this->now();

            geometry_msgs::msg::PoseStamped rbt_pose;
            rbt_pose.pose.position.x = rbt_x;
            rbt_pose.pose.position.y = rbt_y;
            rbt_pose.header.frame_id = this->frame_id_map_;
            rbt_pose.header.stamp = this->now();

            auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
            request->goal = waypoint_pose;
            request->start = rbt_pose;
            this->request_plan_future_ =
                this->cli_plan_->async_send_request(
                                   request,
                                   std::bind(&Behavior::callbackCliReceiveGetPlan_, this, std::placeholders::_1))
                    .future;
        }

        void callbackCliReceiveGetPlan_(const rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
        {
            (void)future;
            // RCLCPP_INFO_STREAM(this->get_logger(), "Plan received");
            // future.get()->plan;
            this->plan_requested_ = false;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::turtle2::Behavior);