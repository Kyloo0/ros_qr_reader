#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("get_qr_direction")
    {
        subscription_direction_ = this->create_subscription<std_msgs::msg::String>(
            "qr_direction", 10,
            std::bind(&MinimalSubscriber::direction_listener_callback, this, std::placeholders::_1));
        
        subscription_target_ = this->create_subscription<std_msgs::msg::Int8>(
            "qr_target", 10,
            std::bind(&MinimalSubscriber::target_listener_callback, this, std::placeholders::_1));
    }

private:
    void direction_listener_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Direction: \"%s\"", msg->data.c_str());

        if (msg->data == "N") {
            RCLCPP_INFO(this->get_logger(), "to North");
        } else if (msg->data == "S") {
            RCLCPP_INFO(this->get_logger(), "to South");
        } else if (msg->data == "E") {
            RCLCPP_INFO(this->get_logger(), "to East");
        } else if (msg->data == "W") {
            RCLCPP_INFO(this->get_logger(), "to West");
        }
    }


    void target_listener_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Target: %d", msg->data);
    } 

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_direction_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_target_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
