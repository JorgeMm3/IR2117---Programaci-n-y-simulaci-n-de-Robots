#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int min;
std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    min +=  msg->data;
    std_msgs::msg::Int32 out_msg;
    out_msg.data = min;
    publisher->publish(out_msg);
}

int main(int argc, char * argv[])
{
    min = 0;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sum");
    auto subcription = node ->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    publisher = node->create_publisher<std_msgs::msg::Int32>("sum", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}