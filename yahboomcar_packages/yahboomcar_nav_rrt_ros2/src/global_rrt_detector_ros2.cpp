#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "functions.h"
#include "mtrand.h"
#include <vector>

class GlobalRRTDetector : public rclcpp::Node
{
public:
    GlobalRRTDetector() : Node("global_rrt_detector")
    {
        // Declare parameters
        this->declare_parameter("eta", 0.5);
        this->declare_parameter("map_topic", "/map");

        eta_ = this->get_parameter("eta").as_double();
        std::string map_topic = this->get_parameter("map_topic").as_string();

        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, 10, std::bind(&GlobalRRTDetector::mapCallback, this, std::placeholders::_1));

        clicked_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&GlobalRRTDetector::clickedCallback, this, std::placeholders::_1));

        // Publishers
        targets_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_points", 10);
        shapes_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("shapes", 10);

        RCLCPP_INFO(this->get_logger(), "Global RRT detector initialized");
    }

    void run()
    {
        // Wait for map
        while (mapData_.data.empty() && rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        // Initialize markers
        points_.header.frame_id = mapData_.header.frame_id;
        line_.header.frame_id = mapData_.header.frame_id;
        points_.ns = line_.ns = "markers";
        points_.id = 0;
        line_.id = 1;
        points_.type = visualization_msgs::msg::Marker::POINTS;
        line_.type = visualization_msgs::msg::Marker::LINE_LIST;
        points_.action = line_.action = visualization_msgs::msg::Marker::ADD;
        points_.pose.orientation.w = line_.pose.orientation.w = 1.0;
        line_.scale.x = line_.scale.y = 0.03;
        points_.scale.x = points_.scale.y = 0.3;
        line_.color.r = 9.0/255.0;
        line_.color.g = 91.0/255.0;
        line_.color.b = 236.0/255.0;
        points_.color.r = 1.0;
        points_.color.a = line_.color.a = 1.0;

        // Wait for initial points
        while (points_.points.size() < 5 && rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            shapes_pub_->publish(points_);
        }

        // Calculate map dimensions
        float init_map_x = std::abs(points_.points[2].x - points_.points[0].x);
        float init_map_y = std::abs(points_.points[2].y - points_.points[0].y);
        float Xstartx = (points_.points[0].x + points_.points[2].x) * 0.5;
        float Xstarty = (points_.points[0].y + points_.points[2].y) * 0.5;

        // Initialize RRT
        std::vector<std::vector<float>> V;
        std::vector<float> xnew;
        xnew.push_back(points_.points[4].x);
        xnew.push_back(points_.points[4].y);
        V.push_back(xnew);
        points_.points.clear();
        shapes_pub_->publish(points_);

        MTRand drand;
        auto rate = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this, &V, &drand, init_map_x, init_map_y, Xstartx, Xstarty]() {
                rrtLoop(V, drand, init_map_x, init_map_y, Xstartx, Xstarty);
            });

        rclcpp::spin(this->shared_from_this());
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        mapData_ = *msg;
    }

    void clickedCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        geometry_msgs::msg::Point p;
        p.x = msg->point.x;
        p.y = msg->point.y;
        p.z = msg->point.z;
        points_.points.push_back(p);
    }

    void rrtLoop(std::vector<std::vector<float>>& V, MTRand& drand,
                 float init_map_x, float init_map_y, float Xstartx, float Xstarty)
    {
        std::vector<float> x_rand, x_nearest, x_new;

        // Sample free
        float xr = (drand() * init_map_x) - (init_map_x * 0.5) + Xstartx;
        float yr = (drand() * init_map_y) - (init_map_y * 0.5) + Xstarty;
        x_rand.push_back(xr);
        x_rand.push_back(yr);

        // Nearest
        x_nearest = Nearest(V, x_rand);

        // Steer
        x_new = Steer(x_nearest, x_rand, eta_);

        // ObstacleFree
        char checking = ObstacleFree(x_nearest, x_new, mapData_);

        if (checking == -1) {
            // Frontier found
            geometry_msgs::msg::PointStamped exploration_goal;
            exploration_goal.header.stamp = this->now();
            exploration_goal.header.frame_id = mapData_.header.frame_id;
            exploration_goal.point.x = x_new[0];
            exploration_goal.point.y = x_new[1];
            exploration_goal.point.z = 0.0;

            geometry_msgs::msg::Point p;
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            points_.points.push_back(p);
            shapes_pub_->publish(points_);
            targets_pub_->publish(exploration_goal);
            points_.points.clear();
        }
        else if (checking == 1) {
            // Free space
            V.push_back(x_new);

            geometry_msgs::msg::Point p;
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            line_.points.push_back(p);
            p.x = x_nearest[0];
            p.y = x_nearest[1];
            line_.points.push_back(p);
        }

        shapes_pub_->publish(line_);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr targets_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shapes_pub_;

    nav_msgs::msg::OccupancyGrid mapData_;
    visualization_msgs::msg::Marker points_, line_;
    float eta_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalRRTDetector>();
    node->run();
    rclcpp::shutdown();
    return 0;
}



