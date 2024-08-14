#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/order.hpp"  
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "yaml-cpp/yaml.h"
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>

class OrderOptimizerNode : public rclcpp::Node {
public:
    OrderOptimizerNode() : Node("order_optimizer_node") {
        this->declare_parameter<std::string>("directory_path", "/home/rahul/Downloads/amr_example_ROS/applicants_amr_example_1");
        this->get_parameter("directory_path", directory_path_);
        
        orders_folder_path_ = directory_path_ + "/orders";
        config_folder_path_ = directory_path_ + "/configuration";


        order_subscription_ = this->create_subscription<interfaces::msg::Order>(
            "nextOrder", 10, std::bind(&OrderOptimizerNode::orderCallback, this, std::placeholders::_1));
        
        // position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "currentPosition", 10, std::bind(&OrderOptimizerNode::positionCallback, this, std::placeholders::_2));
    }

private:
    struct OrderInfo {
        double cx;
        double cy;
        std::vector<int> products;
    };

    void orderCallback(const interfaces::msg::Order::SharedPtr msg) {
        uint32_t order_id = msg->order_id;  
        loadYAMLFiles(orders_folder_path_, order_id);
    }

    // void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    //     uint32_t x = pose->pose.position.x;  
    // }

    void loadYAMLFiles(const std::string& file_path, uint32_t target_order_id)
    {
        std::vector<std::string> yaml_file_paths;
        for (const auto& entry : std::filesystem::directory_iterator(file_path))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".yaml")
            {
                yaml_file_paths.push_back(entry.path().string());
            }
        }

        for (const auto& yaml_path : yaml_file_paths)
        {
            try
            {
                YAML::Node order_yaml = YAML::LoadFile(yaml_path);
                
                if (order_yaml.IsSequence())
                {
                    for (const auto& item : order_yaml)
                    {
                        if (item.IsMap() && item["order"])
                        {
                            uint32_t order_id = item["order"].as<uint32_t>();
                            if (order_id == target_order_id)
                            {
                                double dest_x = item["cx"].as<double>();
                                double dest_y = item["cy"].as<double>();
                                std::vector<int> products = item["products"].as<std::vector<int>>();

                                RCLCPP_INFO(this->get_logger(), "Order ID %d found in file: %s", target_order_id, yaml_path.c_str());
                                RCLCPP_INFO(this->get_logger(), "dest_x: %f, dest_y: %f", dest_x, dest_y);

                                // Log products
                                RCLCPP_INFO(this->get_logger(), "Products:");
                                for (int product : products)
                                {
                                    RCLCPP_INFO(this->get_logger(), "  %d", product);
                                }
                            }
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "No 'order' key found in an item in YAML file: %s", yaml_path.c_str());
                        }
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Root node is not a sequence in YAML file: %s", yaml_path.c_str());
                }
            }
            catch (const YAML::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error loading YAML file %s: %s", yaml_path.c_str(), e.what());
            }
        }

        RCLCPP_INFO(this->get_logger(), "Finished processing YAML files.");
    }

   

    std::string directory_path_;
    std::string orders_folder_path_;
    std::string config_folder_path_;
    rclcpp::Subscription<interfaces::msg::Order>::SharedPtr order_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrderOptimizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
