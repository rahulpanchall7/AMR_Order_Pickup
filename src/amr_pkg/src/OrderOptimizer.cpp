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
#include <algorithm>
#include <limits>

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

struct OrderInfo {
    double delivery_x;
    double delivery_y;
    std::vector<PartInfo> parts;
};

struct Position {
    double x;
    double y;
};

std::map<uint32_t, OrderInfo> orders_info_;

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
        
        position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "currentPosition", 10, std::bind(&OrderOptimizerNode::positionCallback, this, std::placeholders::_1));
    }

private:

    void orderCallback(const interfaces::msg::Order::SharedPtr msg) {
        uint32_t order_id = msg->order_id;
        
        // Clear the previous orders info
        orders_info_.clear();

        loadOrderFiles(orders_folder_path_, order_id);

        // Process each order
        for (const auto& [order_id, order_info] : orders_info_) {
            std::vector<PartInfo> route = solveTSP(order_info.parts, order_info.delivery_x, order_info.delivery_y);
            
            if (!is_first_order_) {
                Position next_start_location = findNearestStartLocation({amr_position_x_, amr_position_y_});
                RCLCPP_INFO(this->get_logger(), "Moving to next order start location X = %f, Y = %f", next_start_location.x, next_start_location.y);
                // Implement movement code to the next start location
            }
            
            printPathDescription(order_id, route);
        }
    
        // After processing, set the flag to false as the first order is now processed
        is_first_order_ = false;
    }

    void positionCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> pose) {
        amr_position_x_ = pose->pose.position.x;
        amr_position_y_ = pose->pose.position.y;
        RCLCPP_INFO(this->get_logger(), "Current AMR Position: X = %f, Y = %f", amr_position_x_, amr_position_y_);
    }

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
                            OrderInfo order_info;
                            order_info.delivery_x = item["cx"].as<double>();
                            order_info.delivery_y = item["cy"].as<double>();
                            std::vector<uint32_t> product_ids = item["products"].as<std::vector<uint32_t>>();

                            for (const auto& product_id : product_ids) {
                                auto it = std::find_if(product_parts_.begin(), product_parts_.end(),
                                    [product_id](const auto& pair) {
                                        return pair.second.product_id == product_id;
                                    });

                                if (it != product_parts_.end()) {
                                    const ProductInfo& product_info = it->second;

                                    for (const auto& part : product_info.parts) {
                                        order_info.parts.push_back(part);
                                    }
                                }
                            }

                            orders_info_[target_order_id] = order_info;
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

            RCLCPP_INFO(this->get_logger(), "Loaded configuration from %s", config_path.c_str());

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading configuration file %s: %s", config_path.c_str(), e.what());
        }
    }

    double calculateEucDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    } 

    double pathLength(const std::vector<PartInfo>& path) {
        double total_distance = 0.0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            total_distance += calculateEucDistance(
                path[i].pick_x, path[i].pick_y,
                path[i+1].pick_x, path[i+1].pick_y
            );
        }
        return total_distance;
    }  

    std::vector<PartInfo> solveTSP(const std::vector<PartInfo>& parts, double delivery_x, double delivery_y) {
        std::vector<PartInfo> route;
        std::vector<bool> visited(parts.size(), false);
        
        double current_x = amr_position_x_;
        double current_y = amr_position_y_;

        for (size_t i = 0; i < parts.size(); ++i) {
            double nearest_dist = std::numeric_limits<double>::infinity();
            size_t nearest_index = std::numeric_limits<size_t>::max();

            for (size_t j = 0; j < parts.size(); ++j) {
                if (!visited[j]) {
                    double dist = calculateEucDistance(current_x, current_y, parts[j].pick_x, parts[j].pick_y);
                    if (dist < nearest_dist) {
                        nearest_dist = dist;
                        nearest_index = j;
                    }
                }
            }

            if (nearest_index != std::numeric_limits<size_t>::max()) {
                visited[nearest_index] = true;
                const PartInfo& nearest_part = parts[nearest_index];
                route.push_back(nearest_part);
                current_x = nearest_part.pick_x;
                current_y = nearest_part.pick_y;
            }
        }

        route.push_back({ "Delivery", delivery_x, delivery_y });
        
        return route;
    }

    void followPath(PartInfo part) {
        if (part.part != "Delivery")
        {
            RCLCPP_INFO(this->get_logger(), "Moving to X = %f, Y = %f", 
                            part.pick_x, part.pick_y);
        }
        
        // Implement movement code
    }
    
    Position findNearestStartLocation(const Position& current_position) 
    {
        double min_distance = std::numeric_limits<double>::infinity();
        Position nearest_location = {0.0, 0.0};

        for (const auto& [order_id, order_info] : orders_info_) {
            for (const auto& part : order_info.parts) {
                double distance = calculateEucDistance(current_position.x, current_position.y, part.pick_x, part.pick_y);
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest_location = {part.pick_x, part.pick_y};
                }
            }
        }

        return nearest_location;
    }
    
    void printPathDescription(uint32_t order_id, const std::vector<PartInfo>& path) {
        RCLCPP_INFO(this->get_logger(), "Working on order %u", order_id);

        int step = 1;
        for (const auto& part : path) {
            if (part.part == "Delivery") {
                RCLCPP_INFO(this->get_logger(), "%d. Delivering to destination x: %f, y: %f",
                            step++, part.pick_x, part.pick_y);
                followPath(part);
            } else {
                RCLCPP_INFO(this->get_logger(), "%d. Fetching part '%s' at x: %f, y: %f",
                            step++, part.part.c_str(), part.pick_x, part.pick_y);
                followPath(part);
            }
        }
    }

    bool is_first_order_ = true;
    
    double amr_position_x_ = 0.0;
    double amr_position_y_ = 0.0;
   
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
