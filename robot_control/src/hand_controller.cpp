#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class HandController : public rclcpp::Node {
public:
    HandController() : Node("hand_controller") {

        sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hand/joints", 10,
            std::bind(&HandController::callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Hand Controller Started");
    }

private:
    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received joint command size: %zu", msg->data.size());
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandController>());
    rclcpp::shutdown();
    return 0;
}