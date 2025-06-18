#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/point.hpp"

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

        subscription_center_ = this->create_subscription<geometry_msgs::msg::Point>(
            "qr_center_bottom", 10,
            std::bind(&MinimalSubscriber::center_listener_callback, this, std::placeholders::_1));
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

    void center_listener_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Center point received: x=%.2f, y=%.2f, z=%.2f",
                    msg->x, msg->y, msg->z);
        
    }

    void target_listener_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Target: %d", msg->data);
    } 

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_direction_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_center_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_target_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
