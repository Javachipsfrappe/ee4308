#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "ee4308_turtle2/core.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee4308::turtle2
{
    class Planner : public rclcpp::Node
    {

        // publishers and subscribers
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_global_costmap_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr service_plan_;
        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
        int max_access_cost_;

    public:
        explicit Planner(const rclcpp::NodeOptions &options, const std::string &name = "planner")
            : rclcpp::Node(name, options)
        {
            // params
            this->costmap_ = nullptr;
            this->max_access_cost_ = getParameter<int>(this, "max_access_cost", 90).as_int();

            // topics
            pub_path_ = create_publisher<nav_msgs::msg::Path>("plan", rclcpp::ServicesQoS());

            auto qos = rclcpp::ServicesQoS();
            qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); // for getting the latched map message.
            this->sub_global_costmap_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                "global_costmap",
                qos,
                std::bind(&Planner::callbackSubGlobalCostmap_, this, std::placeholders::_1));

            // services
            this->service_plan_ = create_service<nav_msgs::srv::GetPlan>(
                "get_plan",
                std::bind(&Planner::callbackSrvGetPlan_, this, std::placeholders::_1, std::placeholders::_2),
                rclcpp::ServicesQoS());
        }

    private:
        template <typename T>
        rclcpp::Parameter getParameter_(const std::string &name, const T &default_value)
        {
            this->declare_parameter(name, default_value);
            rclcpp::Parameter parameter = this->get_parameter(name);
            // RCLCPP_INFO_STREAM(this->get_logger(),
            //                    "[Parameter] " << name << ": " << parameter);
            return parameter;
        }

        void callbackSubGlobalCostmap_(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            this->costmap_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);
        }

        void callbackSrvGetPlan_(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                                 std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
        {
            if (costmap_ == nullptr)
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "No path returned because global_costmap has not been published.");
                response->plan.header.frame_id = "map";
                response->plan.header.stamp = this->now();
                response->plan.poses.clear();
                return;
            }

            // RCLCPP_INFO_STREAM(this->get_logger(), "Plan request");
            response->plan.header.frame_id = "map";
            response->plan.header.stamp = this->now();

            // find the shortest path and store the path within the class.
            // std::cout << "Running Planner..." << std::endl;
            nav_msgs::msg::Path path = this->plan_(request->start, request->goal);
            // std::cout << "Run complete." << std::endl;

            // publish the path
            pub_path_->publish(path);

            // respond with the path
            response->plan = path;
        }

        nav_msgs::msg::Path plan_(geometry_msgs::msg::PoseStamped F, geometry_msgs::msg::PoseStamped I)
        {
            // prepare plan
            nav_msgs::msg::Path P;
            P.header.frame_id = "map";

            // lambdas
            auto OOM = [this](const int &mx, const int &my) -> bool
            { return mx >= (int)this->costmap_->info.width && my >= (int)this->costmap_->info.height; };
            auto D = [](const int &mx1, const int &my1, const int &mx2, const int &my2) -> double
            { return std::hypot((double)mx2 - mx1, (double)my2 - my1); };
            auto W2M = [this](const double &wx, const double &wy, int &mx, int &my) -> void
            {
                mx = (wx - this->costmap_->info.origin.position.x) / this->costmap_->info.resolution;
                my = (wy - this->costmap_->info.origin.position.y) / this->costmap_->info.resolution;
            };
            auto M2W = [this](double &wx, double &wy, const int &mx, const int &my) -> void
            {
                wx = mx * this->costmap_->info.resolution + this->costmap_->info.origin.position.x;
                wy = my * this->costmap_->info.resolution + this->costmap_->info.origin.position.y;
            };
            auto K = [this](const int &mx, const int &my) -> int
            { return my * (int)this->costmap_->info.width + mx; };
            auto XY = [this](const int &k, int &mx, int &my) -> void
            {
                my = k / (int)this->costmap_->info.width;
                mx = k - my * this->costmap_->info.width;
            };
            auto M = [&](const int &k) -> int
            { return this->costmap_->data[k]; };
            std::vector<std::tuple<double, int, int>> NN(this->costmap_->data.size(), {1e4, -1, 1});
            struct QC
            {
                bool operator()(const std::tuple<int, int> &q1, const std::tuple<int, int> &q2) const { return std::get<0>(q1) > std::get<0>(q2); }
            };
            std::priority_queue<std::tuple<int, int>, std::deque<std::tuple<int, int>>, QC> Q;

            int mxi, myi, mxf, myf;
            W2M(I.pose.position.x, I.pose.position.y, mxi, myi);
            W2M(F.pose.position.x, F.pose.position.y, mxf, myf);
            std::get<0>(NN[K(mxi, myi)]) = 0;
            Q.push({0, K(mxi, myi)});

            // main loop
            while (rclcpp::ok() && !Q.empty())
            {
                // RCLCPP_INFO(this->get_logger(), "Q %ld", Q.size());
                int mx, my, k = std::get<1>(Q.top());
                XY(k, mx, my);
                auto *n = &NN[k];
                Q.pop();
                // RCLCPP_INFO(this->get_logger(), "Expanded %d, %d", mx, my);

                if (std::get<2>(*n) > 1)
                    continue;
                else
                    std::get<2>(*n) = 2;

                if (mx == mxf && my == myf)
                {
                    // RCLCPP_INFO(this->get_logger(), "apath");
                    while (rclcpp::ok() && k >= 0)
                    {
                        n = &NN[k];
                        XY(k, mx, my);
                        geometry_msgs::msg::PoseStamped S;
                        M2W(S.pose.position.x, S.pose.position.y, mx, my);
                        P.poses.push_back(S);
                        k = std::get<1>(*n);
                    }
                    I.header.frame_id = "";
                    I.header.stamp = rclcpp::Time();
                    P.poses.push_back(I);
                    P.header.stamp = this->now();
                    return P;
                }

                constexpr std::array<int, 16> R = {1, 1, 0, -1, -1, -1, 0, 1, 0, 1, 1, 1, 0, -1, -1, -1};
                auto it = R.begin();
                auto it2 = it;
                std::advance(it2, R.size() / 2);
                while (rclcpp::ok() && it2 != R.end())
                {
                    int bx = mx + *it;
                    int by = my + *it2;
                    // RCLCPP_INFO(this->get_logger(), "nb: %d, %d, %d, %d", bx, by, *it, *it2);

                    if (!OOM(bx, by))
                    {
                        // RCLCPP_INFO(this->get_logger(), "Inmap");
                        int bk = K(bx, by);
                        auto *b = &NN[bk];
                        // RCLCPP_INFO(this->get_logger(), "Test");
                        if (std::get<2>(*b) <= 1)
                        {
                            double nd = std::get<0>(*n) + D(bx, by, mx, my) + M(bk);
                            if (M(bk) <= this->max_access_cost_ && nd < std::get<0>(*b))
                            {
                                // RCLCPP_INFO(this->get_logger(), "cheaper");
                                std::get<0>(*b) = nd;
                                std::get<1>(*b) = k;
                                Q.push(std::make_tuple(std::get<0>(*b) + D(mxf, myf, bx, by), bk));
                            }
                        }
                    }
                    it = std::next(it);
                    it2 = std::next(it2);
                }
                // RCLCPP_INFO(this->get_logger(), "----------");
            }
            RCLCPP_WARN_STREAM(this->get_logger(), "No path found!");
            P.header.stamp = this->now();
            return P;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::turtle2::Planner);
