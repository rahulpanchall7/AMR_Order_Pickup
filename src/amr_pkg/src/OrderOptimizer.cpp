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

struct PartInfo {
    std::string part;
    double pick_x;
    double pick_y;
};

struct ProductInfo {
    std::uint32_t product_id;
    std::string product;
    std::vector<PartInfo> parts;
};

class OrderOptimizerNode : public rclcpp::Node {
public:
    OrderOptimizerNode() : Node("order_optimizer_node") {
        this->declare_parameter<std::string>("directory_path", "/home/rahul/Downloads/amr_example_ROS/applicants_amr_example_1");
        this->get_parameter("directory_path", directory_path_);
        
        orders_folder_path_ = directory_path_ + "/orders";
        config_folder_path_ = directory_path_ + "/configuration";

        loadProductConfig(config_folder_path_);

        order_subscription_ = this->create_subscription<interfaces::msg::Order>(
            "nextOrder", 10, std::bind(&OrderOptimizerNode::orderCallback, this, std::placeholders::_1));
        
        // position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "currentPosition", 10, std::bind(&OrderOptimizerNode::positionCallback, this, std::placeholders::_2));
    }

private:

    void orderCallback(const interfaces::msg::Order::SharedPtr msg) {
        uint32_t order_id = msg->order_id;  
        loadOrderFiles(orders_folder_path_, order_id);
    }

    // void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    //     uint32_t x = pose->pose.position.x;  
    // }

    void loadOrderFiles(const std::string& file_path, uint32_t target_order_id)
    {
        std::vector<std::string> yaml_file_paths;
        for (const auto& entry : std::filesystem::directory_iterator(file_path))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".yaml")
            {
                yaml_file_paths.push_back(entry.path().string());
            }
        }

        std::vector<std::future<void>> futures;

        for (const auto& yaml_path : yaml_file_paths)
        {
            futures.push_back(std::async(std::launch::async, [this, yaml_path, target_order_id]()
            {
                processFile(yaml_path, target_order_id);
            }));
        }

        for (auto& future : futures)
        {
            future.get();
        }
        RCLCPP_INFO(this->get_logger(), "Finished processing for Order Id: %u", target_order_id);
    }

    void processFile(const std::string& yaml_path, uint32_t target_order_id) 
    {
        try {
            YAML::Node order_yaml = YAML::LoadFile(yaml_path);

            if (order_yaml.IsSequence()) {
                for (const auto& item : order_yaml) {
                    if (item.IsMap() && item["order"]) {
                        uint32_t order_id = item["order"].as<uint32_t>();
                        if (order_id == target_order_id) {
                            double dest_x = item["cx"].as<double>();
                            double dest_y = item["cy"].as<double>();
                            std::vector<uint32_t> product_ids = item["products"].as<std::vector<uint32_t>>();

                            RCLCPP_INFO(this->get_logger(), "Order Id: %d", target_order_id);
                            RCLCPP_INFO(this->get_logger(), "Destination: X = %f, Y = %f", dest_x, dest_y);
                            RCLCPP_INFO(this->get_logger(), "Products and Parts:");

                            for (const auto& product_id : product_ids) {
                                // Search for the product by its ID in the saved product data
                                auto it = std::find_if(product_parts_.begin(), product_parts_.end(),
                                    [product_id](const auto& pair) {
                                        return pair.second.product_id == product_id;
                                    });

                                if (it != product_parts_.end()) {
                                    const ProductInfo& product_info = it->second;
                                    RCLCPP_INFO(this->get_logger(), " Product Id: %u", 
                                                product_info.product_id);
                                    RCLCPP_INFO(this->get_logger(), " Product Name: %s",
                                                product_info.product.c_str());

                                    for (const auto& part : product_info.parts) {
                                        RCLCPP_INFO(this->get_logger(), "    Part: %s, Pick X: %f, Pick Y: %f", 
                                                    part.part.c_str(), part.pick_x, part.pick_y);
                                    }
                                } else {
                                    RCLCPP_WARN(this->get_logger(), "    Product with ID '%u' not found in configuration", product_id);
                                }
                            }
                        }
                    }
                }
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading YAML file %s: %s", yaml_path.c_str(), e.what());
        }
    }




    void loadProductConfig(const std::string& config_path) {
        try {
            std::vector<std::string> yaml_file_paths;
            for (const auto& entry : std::filesystem::directory_iterator(config_path)) {
                if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
                    yaml_file_paths.push_back(entry.path().string());
                }
            }

            for (const auto& yaml_path : yaml_file_paths) {
                YAML::Node product_yaml = YAML::LoadFile(yaml_path);

                if (product_yaml.IsSequence()) {
                    for (const auto& item : product_yaml) {
                        if (item.IsMap()) {
                            ProductInfo product_info;
                            product_info.product_id = item["id"].as<std::uint32_t>();
                            product_info.product = item["product"].as<std::string>();

                            if (item["parts"] && item["parts"].IsSequence()) {
                                for (const auto& part_node : item["parts"]) {
                                    if (part_node.IsMap()) {
                                        PartInfo part_info;
                                        part_info.part = part_node["part"].as<std::string>();
                                        part_info.pick_x = part_node["cx"].as<double>();
                                        part_info.pick_y = part_node["cy"].as<double>();
                                        product_info.parts.push_back(part_info);
                                    }
                                }
                            }

                            // Store the parsed product information in the map
                            product_parts_[product_info.product] = product_info;
                        }
                    }
                }
            }

            // Print the loaded product information
            // for (const auto& [product_name, product_info] : product_parts_) {
            //     RCLCPP_INFO(this->get_logger(), "Product ID: %u, Product Name: %s", 
            //                 product_info.product_id, product_name.c_str());

            //     for (const auto& part : product_info.parts) {
            //         RCLCPP_INFO(this->get_logger(), "    Part: %s, Pick X: %f, Pick Y: %f", 
            //                     part.part.c_str(), part.pick_x, part.pick_y);
            //     }
            // }

            RCLCPP_INFO(this->get_logger(), "Loaded configuration from %s", config_path.c_str());

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading configuration file %s: %s", config_path.c_str(), e.what());
        }
    }

   

    std::string directory_path_;
    std::string orders_folder_path_;
    std::string config_folder_path_;
    std::map<std::string, ProductInfo> product_parts_;
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
