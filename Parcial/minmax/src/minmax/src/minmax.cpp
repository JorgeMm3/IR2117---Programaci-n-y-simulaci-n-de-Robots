#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int min = 0; 
int max = 0;
int aux = 0;
int message_count = 0;
std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{        
    min =  msg->data;
    message_count++;
    std_msgs::msg::Int32 out_msg1;
    std_msgs::msg::Int32 out_msg2;

    while (message_count > 1){    
        aux = msg->data;
        if (min < aux) {
            out_msg1.data = min;
            std::cout << "Min: " << min << std::endl;

        } else {
            out_msg1.data = aux;
            std::cout << "Min: " << aux << std::endl;
        }
        
        if (max > aux){
            out_msg2.data = max;
            std::cout << "Max: " << max << std::endl;
        } else {
            out_msg2.data = aux;
            std::cout << "Max: " << aux << std::endl;

        }
    }

    if (message_count < 1) {
        out_msg1.data = 0;
    }

     if (message_count == 1) {
        out_msg1.data = min;
        out_msg2.data = max;
        std::cout << "Min: " << min << std::endl;
        std::cout << "Max: " << max << std::endl;
    }

    publisher->publish(out_msg1);
    publisher->publish(out_msg2);

  
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minmax");
    auto subcription = node ->create_subscription<std_msgs::msg::Int32>("numbers", 10, topic_callback);
    publisher = node->create_publisher<std_msgs::msg::Int32>("minmax", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}