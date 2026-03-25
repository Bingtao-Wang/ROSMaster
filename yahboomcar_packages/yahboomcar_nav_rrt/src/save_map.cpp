#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <memory>

class SaveMapNode : public rclcpp::Node
{
public:
    SaveMapNode() : Node("save_map")
    {
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_map",
            std::bind(&SaveMapNode::save_map_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Save map service ready");
    }

private:
    void save_map_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        std::string command = "gnome-terminal -- ros2 run nav2_map_server map_saver_cli -f ~/map";
        int result = system(command.c_str());
        response->success = (result == 0);
        response->message = response->success ? "Map saved successfully" : "Failed to save map";
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SaveMapNode>());
    rclcpp::shutdown();
    return 0;
}
