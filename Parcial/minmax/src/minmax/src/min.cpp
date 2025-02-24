#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int min = 0;
int aux = 0;
int message_count = 0;
std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{        
    min =  msg->data;
    message_count++;
    std_msgs::msg::Int32 out_msg;

    while (message_count > 1){    
        aux = msg->data;
        if (min < aux) {
            out_msg.data = min;
            std::cout << "Min: " << min << std::endl;

        } else {
            out_msg.data = aux;
            std::cout << "Min: " << aux << std::endl;
        }
    }

    if (message_count < 1) {
        out_msg.data = 0;
    }

    publisher->publish(out_msg);
  
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("min");
    auto subcription = node ->create_subscription<std_msgs::msg::Int32>("numbers", 10, topic_callback);
    publisher = node->create_publisher<std_msgs::msg::Int32>("min", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}