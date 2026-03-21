#include "ee4308_drone/planner.hpp"

namespace ee4308::drone
{
    Planner::Planner(
        const rclcpp::NodeOptions &options,
        const std::string &name = "planner")
        : Node(name, options)
    {
        // parameters
        this->interpolation_distance_ = ee4308::getParameter<double>(this, "interpolation_distance", 0.1).as_double();

        // topics
        this->pub_plan_ = this->create_publisher<nav_msgs::msg::Path>("plan", rclcpp::ServicesQoS());

        // services
        this->srv_get_plan_ = this->create_service<nav_msgs::srv::GetPlan>(
            "get_plan", std::bind(&Planner::callbackSrvGetPlan_, this, std::placeholders::_1, std::placeholders::_2));
    }

    void Planner::callbackSrvGetPlan_(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        // RCLCPP_INFO(this->get_logger(), "Received path request");
        response->plan.header.frame_id = "map";
        response->plan.header.stamp = this->now();

        double dx = request->goal.pose.position.x - request->start.pose.position.x;
        double dy = request->goal.pose.position.y - request->start.pose.position.y;
        double dz = request->goal.pose.position.z - request->start.pose.position.z;

        double distance = std::hypot(dx, dy, dz);
        
        int steps = std::floor(distance / this->interpolation_distance_); // floored.
        if (this->interpolation_distance_ < ee4308::THRES)
        {
            RCLCPP_WARN(this->get_logger(), "Path will return only start and goal because the interpolation_distance is too small: %f", this->interpolation_distance_);
            steps = 1;
        }
        for (int i = 0; i < steps; ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            double fraction = i * this->interpolation_distance_ / distance;
            pose.pose.position.x = request->start.pose.position.x + fraction * dx;
            pose.pose.position.y = request->start.pose.position.y + fraction * dy;
            pose.pose.position.z = request->start.pose.position.z + fraction * dz;
            response->plan.poses.push_back(pose);
        }
        response->plan.poses.push_back(request->goal);

        // publish the path
        this->pub_plan_->publish(response->plan);
    }

}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Planner);
