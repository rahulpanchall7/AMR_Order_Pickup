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

////////////////////*********************////////////////////

struct PartInfo {
    std::string part;
    double pick_x;
    double pick_y;
    std::string product_name; 
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
        // this->declare_parameter<std::string>("directory_path", "/home/rahul/Downloads/amr_example_ROS/applicants_amr_example_1");
        // this->get_parameter("directory_path", directory_path_);

        this->declare_parameter<std::string>("directory_path", "");
        this->get_parameter("directory_path", directory_path_);

        if (directory_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'directory_path' is not set. Exiting.");
            rclcpp::shutdown();
            return;
        }
        
        orders_folder_path_ = directory_path_ + "/orders";
        config_folder_path_ = directory_path_ + "/configuration";

        loadProductConfig(config_folder_path_);

        // is called only at the begining of the day, before the first order
        position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "currentPosition", 10, std::bind(&OrderOptimizerNode::positionCallback, this, std::placeholders::_1));

        // wait for the first position to be received
        rclcpp::Rate rate(1); 
        while (!is_position_received_ && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for AMR's Start Position...");
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        order_subscription_ = this->create_subscription<interfaces::msg::Order>(
            "nextOrder", 10, std::bind(&OrderOptimizerNode::orderCallback, this, std::placeholders::_1));
        
        amr_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("order_path", 10);

        // initialize the AMR position to (0, 0) before the first order
        // comment this part when position sent from the topic
        // amr_position_x_ = 0.0;
        // amr_position_y_ = 0.0;
        is_first_order_ = true;
    }

private:

    void orderCallback(const interfaces::msg::Order::SharedPtr msg) {
        uint32_t order_id = msg->order_id;
        order_description = msg->description;
        RCLCPP_INFO(this->get_logger(), "Order Id: %u, Description = %s received", 
            order_id, order_description.c_str());

        orders_info_.clear();

        loadOrderFiles(orders_folder_path_, order_id);

        // to generate a route for each order
        for (const auto& [order_id, order_info] : orders_info_) {

            std::vector<PartInfo> route = solveShortestPath(order_info.parts, order_info.delivery_x, order_info.delivery_y, {amr_position_x_, amr_position_y_});
            
            followPath(order_id, route,{last_delivery_x_, last_delivery_y_});
            publishPickupMarkers(route, {order_info.delivery_x, order_info.delivery_y});

            // update AMR position, to the last delivery order
            amr_position_x_ = order_info.delivery_x;
            amr_position_y_ = order_info.delivery_y;

            is_first_order_ = false;
        }
    }

    void positionCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> pose) {
        amr_position_x_ = pose->pose.position.x;
        amr_position_y_ = pose->pose.position.y;
        is_position_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Current AMR Position: X = %f, Y = %f", amr_position_x_, amr_position_y_);
    }

    void loadOrderFiles(const std::string& file_path, uint32_t target_order_id) {
        // this functions loads, all the .yaml files in an array, and then passes on for parsing
        std::vector<std::string> yaml_file_paths;
        for (const auto& entry : std::filesystem::directory_iterator(file_path)) {
            if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
                yaml_file_paths.push_back(entry.path().string());
            }
        }

        std::vector<std::future<void>> futures;
        // for multithread parsing of yaml files
        for (const auto& yaml_path : yaml_file_paths) {
            futures.push_back(std::async(std::launch::async, [this, yaml_path, target_order_id]() {
                processFile(yaml_path, target_order_id);
            }));
        }

        for (auto& future : futures) {
            future.get();
        }
        RCLCPP_INFO(this->get_logger(), "Finished processing for Order Id: %u", target_order_id);
    }

    void processFile(const std::string& yaml_path, uint32_t target_order_id) {
        try {
            YAML::Node order_yaml = YAML::LoadFile(yaml_path);
            order_found = false;

            if (order_yaml.IsSequence()) {
                for (const auto& item : order_yaml) {
                    if (item.IsMap() && item["order"]) {
                        uint32_t order_id = item["order"].as<uint32_t>();
                        if (order_id == target_order_id) {
                            order_found = true;
                            OrderInfo order_info;
                            order_info.delivery_x = item["cx"].as<double>();
                            order_info.delivery_y = item["cy"].as<double>();
                            last_delivery_x_ = order_info.delivery_x;
                            last_delivery_y_ = order_info.delivery_y;
                            std::vector<uint32_t> product_ids = item["products"].as<std::vector<uint32_t>>();

                            // for saving the product id with parts 
                            for (const auto& product_id : product_ids) {
                                auto it = std::find_if(product_parts_.begin(), product_parts_.end(),
                                    [product_id](const auto& pair) {
                                        return pair.second.product_id == product_id;
                                    });

                                if (it != product_parts_.end()) {
                                    const ProductInfo& product_info = it->second;
                                    for (const auto& part : product_info.parts) {
                                        PartInfo enriched_part = part;
                                        enriched_part.product_name = product_info.product; 
                                        order_info.parts.push_back(enriched_part);
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
        // this function is to load all the product & parts related data into data structures
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

                            // to store the parsed product information in the map
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
        // to calculate the euclidean distance between two points
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

        // store the delivery location separately and remove it from the locations list
        PartInfo delivery_location = {"Delivery", delivery_x, delivery_y,""};

        double current_x = start_position.x;
        double current_y = start_position.y;

        // search for nearest pickup locations
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

        // add the delivery location as the final destination
        route.push_back(delivery_location);

      
        double total_distance = 0.0;
        Position last_position = start_position;

        for (const auto& part : route) {
            total_distance += calculateEucDistance(last_position.x, last_position.y, part.pick_x, part.pick_y);
            last_position.x = part.pick_x;
            last_position.y = part.pick_y;
        }

        RCLCPP_INFO(this->get_logger(), "Final route distance: %.2f", total_distance);

        return route;
    }

    void publishAMRMarker(double x, double y) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "amr_position";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.5; 
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 100.0;  
        marker.scale.y = 100.0; 
        marker.scale.z = 100.0; 
        marker.color.a = 1.0;   
        marker.color.r = 1.0;   
        marker.color.g = 0.0;  
        marker.color.b = 0.0;  

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        amr_publisher_->publish(marker_array);
    }


    void publishPickupMarkers(const std::vector<PartInfo>& parts, const Position& delivery_location) {
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 1;

        // Publish markers for pickup locations (Blue)
        for (const auto& part : parts) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "pickup_locations";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = part.pick_x;
            marker.pose.position.y = part.pick_y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 70.0;
            marker.scale.y = 70.0;
            marker.scale.z = 70.0;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;  // Blue color

            marker_array.markers.push_back(marker);
        }

        // publish marker for the delivery location (Yellow)
        visualization_msgs::msg::Marker delivery_marker;
        delivery_marker.header.frame_id = "map";
        delivery_marker.header.stamp = this->get_clock()->now();
        delivery_marker.ns = "delivery_location";
        delivery_marker.id = marker_id++;
        delivery_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        delivery_marker.action = visualization_msgs::msg::Marker::ADD;
        delivery_marker.pose.position.x = delivery_location.x;
        delivery_marker.pose.position.y = delivery_location.y;
        delivery_marker.pose.position.z = 0.0;
        delivery_marker.pose.orientation.x = 0.0;
        delivery_marker.pose.orientation.y = 0.0;
        delivery_marker.pose.orientation.z = 0.0;
        delivery_marker.pose.orientation.w = 1.0;
        delivery_marker.scale.x = 70.0;
        delivery_marker.scale.y = 70.0;
        delivery_marker.scale.z = 70.0;
        delivery_marker.color.a = 1.0;
        delivery_marker.color.r = 1.0;
        delivery_marker.color.g = 1.0;
        delivery_marker.color.b = 0.0;  

        marker_array.markers.push_back(delivery_marker);

        amr_publisher_->publish(marker_array);
    }

    void followPath(uint32_t order_id, const std::vector<PartInfo>& path, const Position& delivery_position) {
        //  this function basically iterates through the route given by ShortestDistanceSolver, and prints accordingly
        RCLCPP_INFO(this->get_logger(), "Working on Order Id: %u", order_id);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));


        for (const auto& part : path) {
            if (part.part != "Delivery") {
                RCLCPP_INFO(this->get_logger(), "Fetching '%s' of '%s' at (%.2f, %.2f)",
                            part.part.c_str(), part.product_name.c_str(), part.pick_x, part.pick_y);
                amr_position_x_ = part.pick_x;
                amr_position_y_ = part.pick_y;
                publishAMRMarker(part.pick_x, part.pick_y);
                publishAMRMarker(part.pick_x, part.pick_y);
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        
        amr_position_x_ = delivery_position.x;
        amr_position_y_ = delivery_position.y;

        publishAMRMarker(delivery_position.x, delivery_position.y);

        publishAMRMarker(delivery_position.x, delivery_position.y);

        RCLCPP_INFO(this->get_logger(), "Delivering to Destination (%.2f, %.2f)", 
                                                    delivery_position.x, delivery_position.y);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        // remove the delivery marker after order completion
        // removeDeliveryMarker();
    }

    // void removeDeliveryMarker() {
    //     visualization_msgs::msg::Marker marker;
    //     marker.header.frame_id = "map";
    //     marker.header.stamp = this->get_clock()->now();
    //     marker.ns = "delivery_location";
    //     marker.id = delivery_marker_id; 
    //     marker.action = visualization_msgs::msg::Marker::DELETE; 
    //     delivery_marker_id = 0;

    //     visualization_msgs::msg::MarkerArray marker_array;
    //     marker_array.markers.push_back(marker);
    //     amr_publisher_->publish(marker_array);
    // }
    
    std::string directory_path_;
    std::string orders_folder_path_;
    std::string config_folder_path_;
    
    std::string order_description;

    rclcpp::Subscription<interfaces::msg::Order>::SharedPtr order_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr amr_publisher_;

    std::map<std::string, ProductInfo> product_parts_;
    double last_delivery_x_ = 0.0;
    double last_delivery_y_ = 0.0;

    double amr_position_x_ = 0.0;
    double amr_position_y_ = 0.0;

    bool is_first_order_ = true;
    bool order_found;
    bool is_position_received_;

    int delivery_marker_id = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrderOptimizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
