#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("square");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    node->declare_parameter("linear_speed", 0.1);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(500ms);
   
    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    
    int i=0, n=4;
    while (rclcpp::ok() && i<n) {
        message.linear.x = linear_speed;
        message.linear.z = 0.0;
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
        i++;
    }

    message.linear.x = 0

    rclcpp::shutdown();
    return 0;
}