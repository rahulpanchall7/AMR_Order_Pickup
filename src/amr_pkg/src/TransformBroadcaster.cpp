#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class TransformBroadcasterNode : public rclcpp::Node {

    public:
        TransformBroadcasterNode() : Node("transform_broadcast") {
            broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            // Example usage in a timer callback or other method
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TransformBroadcasterNode::broadcastTransform, this));
        }

    private:
        void broadcastTransform() {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = this->now();
            transformStamped.header.frame_id = "map";
            transformStamped.child_frame_id = "base_link";
            transformStamped.transform.translation.x = 1.0;
            transformStamped.transform.translation.y = 0.0;
            transformStamped.transform.translation.z = 0.0;
            transformStamped.transform.rotation.x = 0.0;
            transformStamped.transform.rotation.y = 0.0;
            transformStamped.transform.rotation.z = 0.0;
            transformStamped.transform.rotation.w = 1.0;

            broadcaster_->sendTransform(transformStamped);
        }

        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;
    };  

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformBroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}
