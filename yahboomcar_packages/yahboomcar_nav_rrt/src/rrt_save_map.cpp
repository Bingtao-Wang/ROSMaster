#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <memory>

class RRTSaveMap : public rclcpp::Node
{
public:
    RRTSaveMap() : Node("rrt_save_map"), count_(0), if_start_(0)
    {
        this->declare_parameter("rrt_map_name", "rrt_map");
        this->declare_parameter("waiting_time", 30);
        this->declare_parameter("robot", "X3");

        rrt_map_name_ = this->get_parameter("rrt_map_name").as_string();
        waiting_time_ = this->get_parameter("waiting_time").as_int();
        robot_simulation_ = this->get_parameter("robot").as_string();

        speed_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RRTSaveMap::odom_callback, this, std::placeholders::_1));

        start_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&RRTSaveMap::start_callback, this, std::placeholders::_1));

        back_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&RRTSaveMap::timer_callback, this));
    }

private:
    void start_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        (void)msg;
        if_start_++;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (if_start_ > 4) {
            if (msg->twist.twist.linear.x < 0.005 && msg->twist.twist.angular.z < 0.005) {
                count_++;
            } else {
                count_ = 0;
            }
        }
    }

    void timer_callback()
    {
        if (count_ == waiting_time_) {
            std::string command = "gnome-terminal -- ros2 run nav2_map_server map_saver_cli -f " + rrt_map_name_;
            system(command.c_str());
            system("ros2 node kill /assigner");
        }
        if (count_ == (waiting_time_ + 5)) {
            auto origin = geometry_msgs::msg::PoseStamped();
            origin.header.frame_id = "map";
            origin.header.stamp = this->get_clock()->now();
            origin.pose.position.x = 0.0;
            origin.pose.position.y = 0.0;
            origin.pose.position.z = 0.0;
            origin.pose.orientation.w = 1.0;
            back_pub_->publish(origin);
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "count=%d", count_);
    }

    int count_;
    int if_start_;
    int waiting_time_;
    std::string rrt_map_name_;
    std::string robot_simulation_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr back_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTSaveMap>());
    rclcpp::shutdown();
    return 0;
}
