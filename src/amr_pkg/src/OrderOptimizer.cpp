#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/order.hpp"  
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
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

        amr_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("order_path", 10);
    }

private:

    void orderCallback(const interfaces::msg::Order::SharedPtr msg) {
        uint32_t order_id = msg->order_id;
        order_description = msg->description;

        // Clear the previous orders info
        orders_info_.clear();

        // Load the order files for the given order_id
        loadOrderFiles(orders_folder_path_, order_id);

        // Set the initial start position based on whether this is the first order or not
        Position start_position = is_first_order_ ? Position{amr_position_x_, amr_position_y_} : Position{last_delivery_x_, last_delivery_y_};

        // For each order, process and generate a route
        for (const auto& [order_id, order_info] : orders_info_) {
            // Generate the shortest path for the order
            std::vector<PartInfo> route = solveShortestPath(order_info.parts, order_info.delivery_x, order_info.delivery_y, start_position);

            // Print the path description
            printPathDescription(order_id, route);

            // Publish markers for the route
            // publishMarkers(start_position, route);
            publishMarkers(route);

            // Update the last delivery location for the next order
            last_delivery_x_ = order_info.delivery_x;
            last_delivery_y_ = order_info.delivery_y;

            // Update the start position for the next order based on the current delivery
            start_position = {last_delivery_x_, last_delivery_y_};
        }

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

    std::vector<PartInfo> solveShortestPath(const std::vector<PartInfo>& parts, double delivery_x, double delivery_y, const Position& start_position) {
        std::vector<PartInfo> route;
        std::vector<PartInfo> locations = parts;

        double current_x = start_position.x;
        double current_y = start_position.y;

        // Pick up all the parts first
        while (!locations.empty()) {
            double nearest_dist = std::numeric_limits<double>::infinity();
            size_t nearest_index = std::numeric_limits<size_t>::max();

            for (size_t i = 0; i < locations.size(); ++i) {
                double dist = calculateEucDistance(current_x, current_y, locations[i].pick_x, locations[i].pick_y);
                if (dist < nearest_dist) {
                    nearest_dist = dist;
                    nearest_index = i;
                }
            }

            if (nearest_index != std::numeric_limits<size_t>::max()) {
                const PartInfo& nearest_part = locations[nearest_index];
                route.push_back(nearest_part);
                current_x = nearest_part.pick_x;
                current_y = nearest_part.pick_y;
                locations.erase(locations.begin() + nearest_index);
            }
        }

        // After all parts are picked, add the delivery location
        route.push_back({"Delivery", delivery_x, delivery_y});

        return route;
    }

    void publishMarkers(const std::vector<PartInfo>& path) {
        visualization_msgs::msg::MarkerArray marker_array;
        // const Position& start_position,
        // Create a marker for the AMR position
        visualization_msgs::msg::Marker amr_marker;
        amr_marker.header.frame_id = "map";
        amr_marker.header.stamp = this->get_clock()->now();
        amr_marker.ns = "order_path";
        amr_marker.id = 0;
        amr_marker.type = visualization_msgs::msg::Marker::CUBE;
        amr_marker.action = visualization_msgs::msg::Marker::ADD;
        amr_marker.pose.position.x = amr_position_x_;
        amr_marker.pose.position.y = amr_position_y_;
        amr_marker.pose.position.z = 0.0;
        amr_marker.scale.x = 0.5;
        amr_marker.scale.y = 0.5;
        amr_marker.scale.z = 0.2;
        amr_marker.color.a = 1.0;
        amr_marker.color.r = 1.0;
        amr_marker.color.g = 0.0;
        amr_marker.color.b = 0.0;
        marker_array.markers.push_back(amr_marker);

        // Create markers for each part pickup location
        int id = 1;
        for (const auto& part : path) {
            visualization_msgs::msg::Marker part_marker;
            part_marker.header.frame_id = "map";
            part_marker.header.stamp = this->get_clock()->now();
            part_marker.ns = "order_path";
            part_marker.id = id++;
            part_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            part_marker.action = visualization_msgs::msg::Marker::ADD;
            part_marker.pose.position.x = part.pick_x;
            part_marker.pose.position.y = part.pick_y;
            part_marker.pose.position.z = 0.0;
            part_marker.scale.x = 0.3;
            part_marker.scale.y = 0.3;
            part_marker.scale.z = 0.3;
            part_marker.color.a = 1.0;
            part_marker.color.r = 0.0;
            part_marker.color.g = 1.0;
            part_marker.color.b = 0.0;
            marker_array.markers.push_back(part_marker);
        }

        // Publish the marker array
        amr_publisher_->publish(marker_array);
    }

    void followPath(const PartInfo& part) {
        if (part.part != "Delivery") {
            RCLCPP_INFO(this->get_logger(), "Moving to X = %f, Y = %f", 
                            part.pick_x, part.pick_y);
        }
        // Implement movement code
    }
    
    void printPathDescription(uint32_t order_id, const std::vector<PartInfo>& path) {
        RCLCPP_INFO(this->get_logger(), "Working on order %u, %s", order_id, order_description.c_str());

        int step = 1;
        for (const auto& part : path) {
            if (part.part == "Delivery") {
                RCLCPP_INFO(this->get_logger(), "%d. Delivering to destination x: %f, y: %f",
                            step++, part.pick_x, part.pick_y);
            } else {
                RCLCPP_INFO(this->get_logger(), "%d. Fetching part '%s' at x: %f, y: %f",
                            step++, part.part.c_str(), part.pick_x, part.pick_y);
            }
            followPath(part); // Move to the next location
        }

    }

    std::string order_description = "";

    bool is_first_order_ = true;
    
    double amr_position_x_ = 0.0;
    double amr_position_y_ = 0.0;

    double last_delivery_x_ = 0.0;
    double last_delivery_y_ = 0.0;
   
    std::string directory_path_;
    std::string orders_folder_path_;
    std::string config_folder_path_;

    std::map<std::string, ProductInfo> product_parts_;

    rclcpp::Subscription<interfaces::msg::Order>::SharedPtr order_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr amr_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrderOptimizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
