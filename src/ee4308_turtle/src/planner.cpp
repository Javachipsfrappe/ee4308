#include "ee4308_turtle/planner.hpp"

namespace ee4308::turtle
{

    // ====================== Planner Node ===================
    AStarNode::AStarNode(int new_c, int new_r) : c(new_c), r(new_r) {}

    // ======================== Nav2 Planner Plugin ===============================
    void Planner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        // initialize states / variables
        this->node_ = parent.lock(); // this class is not a node. It is instantiated as part of a node `parent`.
        this->tf_ = tf;
        this->plugin_name_ = name;
        this->costmap_ = costmap_ros->getCostmap();
        this->global_frame_id_ = costmap_ros->getGlobalFrameID();

        // declare parameters to let the node know we are using these params.
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_access_cost", this->max_access_cost_, 254);
        ee4308::initParam(this->node_, this->plugin_name_ + ".interpolation_distance", this->interpolation_distance_, 0.05);
        
        // Hardcoded Savitzky-Golay smoothing parameters optimized for tight corners
        this->sg_half_window_ = 3;  // Reduced from 4 for better corner preservation
        this->sg_order_ = 3;        // Polynomial order for smoothing kernel
    }

    // Converts world coordinates to cell column and cell row.
    std::pair<int, int> Planner::XYToCR_(double x, double y)
    {
        // The following functions may be used:
        //   costmap_->getOriginX()
        //   costmap_->getOriginY()
        //   costmap_->getResolution()
        //   std::ceil()
        //   std::floor()

        double resolution = costmap_->getResolution();
        double origin_x = costmap_->getOriginX();
        double origin_y = costmap_->getOriginY();

        // Translate world coordinates to map coordinates relative to origin
        double dx = (x - origin_x) / resolution;
        double dy = (y - origin_y) / resolution;

        // Convert to cell indices (c for column, r for row)
        int c = std::floor(dx);
        int r = std::floor(dy);

        return {c, r};
    }
    
    // Converts cell column and cell row to world coordinates.
    std::pair<double, double> Planner::CRToXY_(int c, int r)
    {
        // The following functions may be used:
        //   this->costmap_->getResolution()
        //   this->costmap_->getOriginX()
        //   this->costmap_->getOriginY()

        double resolution = costmap_->getResolution();
        double origin_x = costmap_->getOriginX();
        double origin_y = costmap_->getOriginY();

        // Convert cell indices to world coordinates (center of cell)
        double x = origin_x + (c + 0.5) * resolution;
        double y = origin_y + (r + 0.5) * resolution;

        return {x, y};
    }


    // Converts cell column and cell row to flattened array index.
    int Planner::CRToIndex_(int c, int r)
    {
        // The following functions may be used:
        //   this->costmap_->getSizeInCellsX()
        //   this->costmap_->getSizeInCellsY()

        // Row-major order: index = r * num_cols + c
        int num_cols = costmap_->getSizeInCellsX();
        return r * num_cols + c;
    }

    // Returns true if out of map, false otherwise.
    bool Planner::outOfMap_(int c, int r)
    {
        // The following functions may be used:
        //   this->costmap_->getSizeInCellsX()
        //   this->costmap_->getSizeInCellsY()

        int num_cols = costmap_->getSizeInCellsX();
        int num_rows = costmap_->getSizeInCellsY();

        return c < 0 || c >= num_cols || r < 0 || r >= num_rows;
    }

    // Compute Savitsky-Golay kernel using Vandermonde matrix
    Eigen::RowVectorXd Planner::computeSGKernel_(int half_window, int order)
    {
        // Clamp order to avoid singular matrix (order must be < window_size)
        order = std::min(order, 2 * half_window);
        
        int window_size = 2 * half_window + 1;
        
        // Create Vandermonde matrix J
        Eigen::MatrixXd J(window_size, order + 1);
        for (int r = 0; r < window_size; ++r) {
            for (int c = 0; c <= order; ++c) {
                J(r, c) = std::pow(-half_window + r, c);
            }
        }

        // Compute A using numerically stable LDLT decomposition
        // Solving (J^T*J)X = J^T is more stable than inverting J^T*J directly
        Eigen::MatrixXd JT = J.transpose();
        Eigen::MatrixXd JtJ = JT * J;
        Eigen::MatrixXd A = JtJ.ldlt().solve(JT);

        // Return the first row of A as RowVectorXd
        return A.row(0);
    }

    // Apply Savitsky-Golay smoothing to path
    nav_msgs::msg::Path Planner::smoothPath_(const nav_msgs::msg::Path &raw_path)
    {
        nav_msgs::msg::Path smoothed_path = raw_path;
        
        // Need at least half_window + 1 points on each side
        if ((int)raw_path.poses.size() < 2 * sg_half_window_ + 1) {
            return raw_path; // Path too short for smoothing
        }

        // Compute kernel
        Eigen::RowVectorXd kernel = computeSGKernel_(sg_half_window_, sg_order_);

        // Extract x and y coordinates
        std::vector<double> x_coords(raw_path.poses.size());
        std::vector<double> y_coords(raw_path.poses.size());
        for (size_t i = 0; i < raw_path.poses.size(); ++i) {
            x_coords[i] = raw_path.poses[i].pose.position.x;
            y_coords[i] = raw_path.poses[i].pose.position.y;
        }

        // Apply smoothing to x coordinates
        std::vector<double> smoothed_x(x_coords.size());
        for (int i = 0; i < (int)x_coords.size(); ++i) {
            double sum = 0.0;
            for (int j = -sg_half_window_; j <= sg_half_window_; ++j) {
                int idx = i + j;
                if (idx < 0) idx = 0;
                if (idx >= (int)x_coords.size()) idx = x_coords.size() - 1;
                sum += kernel(j + sg_half_window_) * x_coords[idx];
            }
            smoothed_x[i] = sum;
        }

        // Apply smoothing to y coordinates
        std::vector<double> smoothed_y(y_coords.size());
        for (int i = 0; i < (int)y_coords.size(); ++i) {
            double sum = 0.0;
            for (int j = -sg_half_window_; j <= sg_half_window_; ++j) {
                int idx = i + j;
                if (idx < 0) idx = 0;
                if (idx >= (int)y_coords.size()) idx = y_coords.size() - 1;
                sum += kernel(j + sg_half_window_) * y_coords[idx];
            }
            smoothed_y[i] = sum;
        }

        // Update poses with smoothed coordinates
        for (size_t i = 0; i < smoothed_path.poses.size(); ++i) {
            smoothed_path.poses[i].pose.position.x = smoothed_x[i];
            smoothed_path.poses[i].pose.position.y = smoothed_y[i];
        }

        return smoothed_path;
    }

    nav_msgs::msg::Path Planner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        std::function<bool()> /*cancel_checker*/)
    {
        // Get map dimensions and resolution
        int num_cols = costmap_->getSizeInCellsX();
        int num_rows = costmap_->getSizeInCellsY();
        double resolution = costmap_->getResolution();

        // Create a vector of nodes
        std::vector<AStarNode> nodes;
        for (int r = 0; r < num_rows; ++r) {
            for (int c = 0; c < num_cols; ++c) {
                nodes.emplace_back(c, r);
            }
        }

        // Initialize all nodes to ensure clean state
        for (auto &n : nodes) {
            n.g = std::numeric_limits<double>::infinity();
            n.h = std::numeric_limits<double>::infinity();
            n.f = std::numeric_limits<double>::infinity();
            n.parent = nullptr;
            n.expanded = false;
        }

        // Create an open list
        OpenList<AStarNode *> open_list;

        // Get the c,r map coordinates of the start and goal points
        auto [start_c, start_r] = this->XYToCR_(start.pose.position.x, start.pose.position.y);
        auto [goal_c, goal_r] = this->XYToCR_(goal.pose.position.x, goal.pose.position.y);

        // Check if start and goal are valid
        if (outOfMap_(start_c, start_r) || outOfMap_(goal_c, goal_r)) {
            RCLCPP_ERROR(this->node_->get_logger(), "Start or goal position is out of map bounds");
            return this->writeToPath_(nullptr, goal); // no path
        }

        // Initialize start node
        int start_idx = CRToIndex_(start_c, start_r);
        AStarNode *start_node = &nodes[start_idx];
        start_node->g = 0.0;
        
        // Use Octile heuristic for 8-connected grid (more accurate than Euclidean)
        int dcg = std::abs(goal_c - start_c);
        int drg = std::abs(goal_r - start_r);
        start_node->h = resolution * (std::max(dcg, drg) + (std::sqrt(2.0) - 1.0) * std::min(dcg, drg));
        
        start_node->f = start_node->h;
        open_list.push(start_node);

        // ================ Expansion loop ========================
        while (rclcpp::ok() && !open_list.empty()) {
            // Pop the cheapest node
            AStarNode *node = open_list.top();
            open_list.pop();

            // Skip if already expanded
            if (node->expanded) {
                continue;
            }

            // Check if goal is reached
            if (goal_c == node->c && goal_r == node->r) {
                return this->writeToPath_(node, goal);
            }

            // Mark as expanded
            node->expanded = true;

            // ================ Neighbor loop ========================
            // 8-connected grid: allowed movements in all directions
            for (auto [dc, dr] : std::vector<std::pair<int, int>>{{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}}) {
                int nb_c = node->c + dc;
                int nb_r = node->r + dr;

                // Check if neighbor is out of bounds
                if (outOfMap_(nb_c, nb_r)) {
                    continue;
                }

                // Get neighbor node
                int nb_idx = CRToIndex_(nb_c, nb_r);
                AStarNode *neighbor = &nodes[nb_idx];

                // Skip if already expanded
                if (neighbor->expanded) {
                    continue;
                }

                // Get cost of neighbor cell
                unsigned char cell_cost = costmap_->getCost(nb_c, nb_r);

                // Check if cell is accessible
                if ((int)cell_cost >= max_access_cost_) {
                    continue;
                }

                // Calculate cost with balanced obstacle penalty for tight corridor navigation
                // Softer penalty avoids rejecting narrow corridors while still avoiding obstacles
                double cost_factor = 1.0 + (double)cell_cost / 150.0;

                // Calculate distance in meters (1.0 or sqrt(2) cells * resolution)
                double distance = resolution * ((dc != 0 && dr != 0) ? std::sqrt(2.0) : 1.0);

                // Calculate new g-cost
                double new_g = node->g + distance * cost_factor;

                // If this path to neighbor is better than any previous one
                if (new_g < neighbor->g) {
                    neighbor->g = new_g;
                    neighbor->parent = node;

                    // Calculate h-cost using Octile heuristic (better for 8-connected grids than Euclidean)
                    int dcg_nb = std::abs(goal_c - nb_c);
                    int drg_nb = std::abs(goal_r - nb_r);
                    neighbor->h = resolution * (std::max(dcg_nb, drg_nb) + (std::sqrt(2.0) - 1.0) * std::min(dcg_nb, drg_nb));
                    neighbor->f = neighbor->g + neighbor->h;

                    // Add to open list
                    open_list.push(neighbor);
                }
            }
        }

        RCLCPP_WARN(this->node_->get_logger(), "No path found to goal");
        return this->writeToPath_(nullptr, goal); // no path
    }

    nav_msgs::msg::Path Planner::writeToPath_(
        AStarNode *goal_node,
        geometry_msgs::msg::PoseStamped goal)
    {
        // setup the path message
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = this->global_frame_id_;
        path.header.stamp = this->node_->now();

        // No path found
        if (goal_node == nullptr) {
            return path;
        }

        // Extract path from goal node back to start using parent pointers
        AStarNode* node = goal_node;
        while (node != nullptr) {
            // Convert map coordinates to world coordinates
            auto [wx, wy] = this->CRToXY_(node->c, node->r);
            
            // Push the pose into the messages
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1; // normalized quaternion
            path.poses.push_back(pose);

            // Go to the next node
            node = node->parent;
        }
        
        // Reverse the path so start is at front and goal is at back
        std::reverse(path.poses.begin(), path.poses.end());

        // Interpolate path to ensure uniform spacing
        nav_msgs::msg::Path interpolated_path;
        interpolated_path.header = path.header;
        
        for (size_t i = 0; i < path.poses.size() - 1; ++i) {
            interpolated_path.poses.push_back(path.poses[i]);

            double x1 = path.poses[i].pose.position.x;
            double y1 = path.poses[i].pose.position.y;
            double x2 = path.poses[i + 1].pose.position.x;
            double y2 = path.poses[i + 1].pose.position.y;

            double distance = ee4308::getDistance(x1, y1, x2, y2);
            int num_interpolated = static_cast<int>(std::ceil(distance / interpolation_distance_)) - 1;

            for (int j = 1; j <= num_interpolated; ++j) {
                double t = static_cast<double>(j) / (num_interpolated + 1);
                geometry_msgs::msg::PoseStamped interp_pose;
                interp_pose.pose.position.x = x1 + t * (x2 - x1);
                interp_pose.pose.position.y = y1 + t * (y2 - y1);
                interp_pose.pose.orientation.w = 1;
                interpolated_path.poses.push_back(interp_pose);
            }
        }
        
        // Add the final goal pose
        interpolated_path.poses.push_back(path.poses.back());

        // Apply Savitsky-Golay smoothing
        nav_msgs::msg::Path smoothed_path = smoothPath_(interpolated_path);

        // Push the original goal (contains the final yaw angle of the robot)
        goal.header.frame_id = "";
        goal.header.stamp = rclcpp::Time(); // prevents nav2 and tf2 from having time extrapolation issues
        smoothed_path.poses.push_back(goal);

        return smoothed_path;
    }

    // ======================================== DO NOT TOUCH =================================

    void Planner::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Planner, nav2_core::GlobalPlanner)