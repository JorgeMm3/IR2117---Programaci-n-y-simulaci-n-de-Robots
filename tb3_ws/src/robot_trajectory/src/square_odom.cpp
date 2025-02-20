#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;

void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

        std::cout << msg << std::endl;
        
}
  
int main(int argc, char * argv[]){
  

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("subscriber");
  auto subscription = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, topic_callback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
