#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/order.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "yaml-cpp/yaml.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"

class OrderOptimizer : public rclcpp::Node 
{
public:
    OrderOptimizer() : Node("order_optimizer")
    {
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrderOptimizer>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}