#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("get_qr_direction")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "qr_direction", 10,
            std::bind(&MinimalSubscriber::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg)
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

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
