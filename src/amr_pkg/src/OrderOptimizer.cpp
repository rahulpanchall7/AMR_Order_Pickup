#include "rclcpp/rclcpp.hpp"
#include "amr_interfaces/msg/order.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <filesystem>

class OrderOptimizer : public rclcpp::Node 
{
public:
    OrderOptimizer() : Node("order_optimizer") 
    {
        
        // declare path directory
        this->declare_parameter<std::string>("path_dir", "/home/rahul/Downloads/amr_example_ROS/");
        path_dir_ = this->get_parameter("path_dir").as_string();

        // initializing subscribers
        position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "currentPosition", 10, 
            std::bind(&OrderOptimizer::callbackCurrentPosition, this, std::placeholders::_1));

        
        order_subscriber_ = this->create_subscription<amr_interfaces::msg::Order>(
            "nextOrder", 10, 
            std::bind(&OrderOptimizer::callbackNextOrder, this, std::placeholders::_1));
    }

private:
    // callbacks for subs
    void callbackCurrentPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Position: x=%f, y=%f, z=%f",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    void callbackNextOrder(const amr_interfaces::msg::Order::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Order: id=%u, description=%s",
                    msg->order_id, msg->description.c_str());
    }

    // declaring subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber_;
    rclcpp::Subscription<amr_interfaces::msg::Order>::SharedPtr order_subscriber_;
    std::string path_dir_;
};  

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrderOptimizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
