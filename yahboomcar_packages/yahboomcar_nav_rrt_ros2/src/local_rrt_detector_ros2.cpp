#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "functions.h"
#include "mtrand.h"
#include <vector>

class LocalRRTDetector : public rclcpp::Node
{
public:
    LocalRRTDetector() : Node("local_rrt_detector")
    {
        this->declare_parameter("eta", 0.5);
        this->declare_parameter("map_topic", "/map");
        this->declare_parameter("range", 5.0);
        this->declare_parameter("robot_frame", "base_link");

        eta_ = this->get_parameter("eta").as_double();
        range_ = this->get_parameter("range").as_double();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        std::string map_topic = this->get_parameter("map_topic").as_string();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, 10, std::bind(&LocalRRTDetector::mapCallback, this, std::placeholders::_1));

        clicked_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&LocalRRTDetector::clickedCallback, this, std::placeholders::_1));

        targets_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_points", 10);
        shapes_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("shapes", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Local RRT detector initialized");
    }

    void run()
    {
        while (mapData_.data.empty() && rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        initMarkers();

        auto timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&LocalRRTDetector::rrtLoop, this));

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

    void initMarkers()
    {
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
        points_.color.r = 1.0;
        points_.color.a = line_.color.a = 1.0;
    }

    void rrtLoop()
    {
        if (!initialized_) {
            try {
                auto transform = tf_buffer_->lookupTransform(
                    mapData_.header.frame_id, robot_frame_, tf2::TimePointZero);
                robot_x_ = transform.transform.translation.x;
                robot_y_ = transform.transform.translation.y;
                initialized_ = true;
            } catch (tf2::TransformException& ex) {
                return;
            }
        }

        MTRand drand;
        std::vector<float> x_rand, x_nearest, x_new;

        float xr = (drand() * range_ * 2) - range_ + robot_x_;
        float yr = (drand() * range_ * 2) - range_ + robot_y_;
        x_rand.push_back(xr);
        x_rand.push_back(yr);

        x_nearest = {robot_x_, robot_y_};
        x_new = Steer(x_nearest, x_rand, eta_);

        int checking = ObstacleFree(x_nearest, x_new, mapData_);

        if (checking == -1) {
            geometry_msgs::msg::PointStamped goal;
            goal.header.stamp = this->now();
            goal.header.frame_id = mapData_.header.frame_id;
            goal.point.x = x_new[0];
            goal.point.y = x_new[1];
            goal.point.z = 0.0;
            targets_pub_->publish(goal);
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr targets_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shapes_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::OccupancyGrid mapData_;
    visualization_msgs::msg::Marker points_, line_;
    float eta_, range_;
    std::string robot_frame_;
    float robot_x_ = 0.0, robot_y_ = 0.0;
    bool initialized_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalRRTDetector>();
    node->run();
    rclcpp::shutdown();
    return 0;
}




