
#include <string>
#include <vector>
#include <memory>
#include <queue>
#include <deque>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "eigen3/Eigen/Dense"

#include "ee4308_turtle/core.hpp"

#pragma once

namespace ee4308::turtle
{
    // ====================== Planner Node ===================
    struct AStarNode
    {
        double f = INFINITY;
        double g = INFINITY;
        double h = INFINITY;
        AStarNode *parent = nullptr;
        int c = -1;
        int r = -1;
        bool expanded = false;

        AStarNode(int new_c, int new_r);
    };

    // ======================= Open List ===========================
    template <typename T> // T must be pointer type
    struct OpenListComparator
    {
        bool operator()(const T &l, const T &r) const { return l->f > r->f; };
    };

    template <typename T> // T must be pointer type
    using OpenList = std::priority_queue<T, std::deque<T>, OpenListComparator<T>>; 

    // ======================== Nav2 Planner Plugin ===============================
    class Planner : public nav2_core::GlobalPlanner
    {
    public:
        Planner() = default;
        ~Planner() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        void cleanup() override;
        void activate() override;
        void deactivate() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal,
            std::function<bool()> cancel_checker) override;

    protected:
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        nav2_costmap_2d::Costmap2D *costmap_; // global costmap
        std::string global_frame_id_;

        // parameters
        int max_access_cost_;
        double interpolation_distance_;
        int sg_half_window_;
        int sg_order_;

        // helper functions for Savitsky-Golay smoothing
        Eigen::RowVectorXd computeSGKernel_(int half_window, int order);
        nav_msgs::msg::Path smoothPath_(const nav_msgs::msg::Path &raw_path);

        std::pair<int, int> XYToCR_(double x, double y);
        int CRToIndex_(int c, int r);
        std::pair<double, double> CRToXY_(int c, int r);
        bool outOfMap_(int c, int r);

        nav_msgs::msg::Path writeToPath_(AStarNode *goal_node, geometry_msgs::msg::PoseStamped goal);
    };

}