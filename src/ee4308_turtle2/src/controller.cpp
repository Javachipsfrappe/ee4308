#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ee4308_turtle2/core.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::turtle2
{
    class Controller : public rclcpp::Node
    {
    private:
        // parameters
        std::string frame_id_turtle_;
        std::string frame_id_map_;
        double frequency_;
        double kp_lin_;
        double kp_ang_;
        double max_lin_vel_;
        double max_ang_vel_;
        double xy_tolerance_;
        double yaw_tolerance_;
        double lookahead_distance_;
        bool enable_;

        // states
        nav_msgs::msg::Path plan_;

        // topics, services, timers
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;              // subscribe to ground truth in simulation.
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_; // subscribe to ground truth in simulation.
        rclcpp::TimerBase::SharedPtr timer_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    public:
        explicit Controller(const rclcpp::NodeOptions &options, const std::string &name = "controller")
            : Node(name, options)
        {
            // parameters
            this->frequency_ = getParameter<double>(this, "frequency", 10.0).as_double();
            this->kp_lin_ = getParameter<double>(this, "kp_lin", 1.0).as_double();
            this->kp_ang_ = getParameter<double>(this, "kp_ang", 1.0).as_double();
            this->max_lin_vel_ = getParameter<double>(this, "max_lin_vel", 0.2).as_double();
            this->max_ang_vel_ = getParameter<double>(this, "max_ang_vel", 1.0).as_double();
            this->xy_tolerance_ = getParameter<double>(this, "xy_tolerance", 0.05).as_double();
            this->yaw_tolerance_ = getParameter<double>(this, "yaw_tolerance", M_PI / 4).as_double();
            this->lookahead_distance_ = getParameter<double>(this, "lookahead_distance", 0.3).as_double();
            this->enable_ = getParameter<bool>(this, "enable", true).as_bool();
            this->frame_id_turtle_ = getParameter<std::string>(this, "frame_id_turtle_", "turtle/base_link").as_string();
            this->frame_id_map_ = getParameter<std::string>(this, "frame_id_map_", "map").as_string();

            // topics
            this->sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
                "plan", rclcpp::SensorDataQoS(),
                std::bind(&Controller::cbSubPlan_, this, std::placeholders::_1));
            this->pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "cmd_vel", rclcpp::ServicesQoS());

            // tf2
            this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Initialize main loop
            this->timer_ = this->create_timer(1s / this->frequency_, std::bind(&Controller::callbackTimer_, this));
        }

    private:
        void cbSubPlan_(const nav_msgs::msg::Path msg)
        {
            plan_ = msg;
        }

        void callbackTimer_()
        {
            if (!enable_)
                return;

            if (plan_.poses.empty())
            {
                this->publishCmdVel_(0, 0);
                return; // no plan.
            }

            // get tf2
            geometry_msgs::msg::TransformStamped tf_turtle;
            try
            {
                tf_turtle = this->tf_buffer_->lookupTransform(
                    this->frame_id_map_, this->frame_id_turtle_,
                    rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(5e7));
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Cannot get tf for turtle: %s", ex.what());
                return;
            }

            double rbt_x = tf_turtle.transform.translation.x;
            double rbt_y = tf_turtle.transform.translation.y;
            double rbt_yaw = getYawFromQuaternion(tf_turtle.transform.rotation);
            // RCLCPP_INFO(this->get_logger(), "%6.3f, %6.3f, %6.3f", rbt_x, rbt_y, rbt_yaw * 180 / M_PI);

            // finds first point from start of path that satisfies lookahead. Uses goal pose if none can be found. assumes robot is slow enough.
            auto exceedsLookahead = [&](const geometry_msgs::msg::PoseStamped &plan_pose)
            { return std::hypot(plan_pose.pose.position.x - rbt_x, plan_pose.pose.position.y - rbt_y) > this->lookahead_distance_; };
            auto it_plan_poses = std::find_if(plan_.poses.begin(), plan_.poses.end(), exceedsLookahead);
            const geometry_msgs::msg::PoseStamped &lookahead_pose = (it_plan_poses == plan_.poses.end()) ? *std::prev(plan_.poses.end()) : *it_plan_poses;
            double dx = lookahead_pose.pose.position.x - rbt_x;
            double dy = lookahead_pose.pose.position.y - rbt_y;

            double err_ang = limitAngle(std::atan2(dy, dx) - rbt_yaw);
            double err_lin = std::hypot(dx, dy);
            double lin_vel = kp_lin_ * err_lin;
            double ang_vel = kp_ang_ * err_ang;

            if (err_lin < this->xy_tolerance_)
            {
                ang_vel = 0;
                lin_vel = 0;
            }

            if (std::abs(err_ang) < this->yaw_tolerance_)
                lin_vel *= (1 - std::abs(err_ang) / this->yaw_tolerance_);
            else
                lin_vel = 0;

            lin_vel = std::clamp(lin_vel, -max_lin_vel_, max_lin_vel_);
            ang_vel = std::clamp(ang_vel, -max_ang_vel_, max_ang_vel_);

            publishCmdVel_(lin_vel, ang_vel);
        }

        void publishCmdVel_(const double &lin_vel, const double &ang_vel)
        {
            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = this->now();
            msg.twist.linear.x = lin_vel;
            msg.twist.angular.z = ang_vel;
            pub_cmd_vel_->publish(msg);
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::turtle2::Controller);