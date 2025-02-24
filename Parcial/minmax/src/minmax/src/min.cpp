#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int min = 0;
int message_count = 0;
std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    min +=  msg->data;
    message_count++;
    std_msgs::msg::Int32 out_msg;
    out_msg.data = min;
    publisher->publish(out_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("min");
    auto subcription = node ->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    publisher = node->create_publisher<std_msgs::msg::Int32>("min", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}