#include <chrono>     
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;  

int main(int argc, char * argv[])  
{
  rclcpp::init(argc, argv);   
  auto node = rclcpp::Node::make_shared("publisher");    
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);     
  node->declare_parameter("linear_speed", 0.1); 
  node->declare_parameter("angular_speed", M_PI / 20);
  node->declare_parameter("square_length", 1.0);
  geometry_msgs::msg::Twist message;   
  
  rclcpp::WallRate loop_rate(10ms);
  int i=0;
  double square_length = node->get_parameter("square_length").get_parameter_value().get<double>();
  double loop_wait = 0.01;
  
  double distance = square_length;
  double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();    // Obtenir el valor del paràmetre
  std::cout << "linear_speed: " << linear_speed << std::endl;
  double linear_iterations = distance / (loop_wait * linear_speed);

  double angle = 90 * M_PI / 180;          
  double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
  std::cout << "angular_speed: " << angular_speed << std::endl;
  double angular_iterations = angle / (loop_wait * angular_speed);

  for(int j=0; j<4; j++)
  {
    i=0;
    
    std::cout << "Avant" << std::endl;
    while (rclcpp::ok() && i<linear_iterations) {
      i++;
      message.linear.x = linear_speed;
      publisher->publish(message);
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
    message.linear.x = 0.0;
    publisher->publish(message);
    
    i=0;

    std::cout << "Gir" << std::endl;
    while (rclcpp::ok() && i<angular_iterations) {
      i++;
      message.angular.z = angular_speed;
      publisher->publish(message);
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }

    message.angular.z = 0.0;
    publisher->publish(message);
  }

  message.linear.x = 0.0;
  message.angular.z = 0.0;
  publisher->publish(message);
  std::cout << "Final del programa" << std::endl;

  rclcpp::shutdown();  
  return 0;
}

