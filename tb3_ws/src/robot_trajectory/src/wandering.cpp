 
#include <chrono>    
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" 
#include "geometry_msgs/msg/twist.hpp"   

using namespace std::chrono_literals;   
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

float min_esquerra;
float min_dreta;

bool turn_left = false;
bool turn_right = false;

void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  //auto moviment = geometry_msgs::msg::Twist();
  min_dreta = msg->ranges[0];
  for (int i = 0; i <= 9; ++i) {
    if (msg->ranges[i] < min_dreta) {
      min_dreta = msg->ranges[i];
    }
  }
  std::cout << "El valor mínim a la dreta és: " << min_dreta << std::endl;

  min_esquerra = msg->ranges[350];
  for (int i = 350; i <= 359; ++i) {
    if (msg->ranges[i] < min_esquerra) {
      min_esquerra = msg->ranges[i];
    }
  }
  std::cout << "El valor mínim a l'esquerra és: " << min_esquerra << std::endl;
}


int main(int argc, char * argv[])    
{
  rclcpp::init(argc, argv);  
  auto node = rclcpp::Node::make_shared("wandering");     
  auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, scan_callback);
  publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  rclcpp::WallRate loop_rate(10ms);  

  while (rclcpp::ok()) {    
    auto moviment = geometry_msgs::msg::Twist();
    
    if (turn_left == false and turn_right == false) {
      moviment.linear.x = 0.5;    
      moviment.angular.z = 0.0;

      if (min_esquerra < 0.5 or min_dreta < 0.5) {   
        if (min_esquerra > min_dreta) {
          turn_left = true;
        } else {
          turn_right = true;
        }
      }

    } else if (turn_left == true) {
      moviment.linear.x = 0.0;
      moviment.angular.z = -0.5;  
      if (min_esquerra > 0.5 and min_dreta > 0.5) {
        turn_left = false;
      }

    } else if (turn_right == true) {
      moviment.linear.x = 0.0;
      moviment.angular.z = 0.5;
      if (min_esquerra > 0.5 and min_dreta > 0.5) {
        turn_right = false;
      }
    }

    publisher->publish(moviment);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();  
  return 0;
}

