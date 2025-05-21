#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode() : Node("monitor")
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Subscripción a detecciones de intrusos
    sub_north_ = this->create_subscription<std_msgs::msg::Bool>("/north", 10, 
      [this](const std_msgs::msg::Bool::SharedPtr msg) { intrusions_["north"] = msg->data; });
    sub_south_ = this->create_subscription<std_msgs::msg::Bool>("/south", 10, 
      [this](const std_msgs::msg::Bool::SharedPtr msg) { intrusions_["south"] = msg->data; });
    sub_east_ = this->create_subscription<std_msgs::msg::Bool>("/east", 10, 
      [this](const std_msgs::msg::Bool::SharedPtr msg) { intrusions_["east"] = msg->data; });
    sub_west_ = this->create_subscription<std_msgs::msg::Bool>("/west", 10, 
      [this](const std_msgs::msg::Bool::SharedPtr msg) { intrusions_["west"] = msg->data; });

    // Odometry (por si se quiere usar más adelante)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
      [this](const nav_msgs::msg::Odometry::SharedPtr) {});

    // Timer para publicar cmd_vel periódicamente
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&MonitorNode::check_intruders, this));
  }

private:
  void check_intruders()
  {
    geometry_msgs::msg::Twist cmd;

    // Lógica simple de respuesta a detección
    int count = 0;

    if (intrusions_["north"])  { cmd.linear.x = 0.5; count++; }
    if (intrusions_["south"])  { cmd.linear.x = -0.5; count++; }
    if (intrusions_["east"])   { cmd.angular.z = -1.0; count++; }
    if (intrusions_["west"])   { cmd.angular.z = 1.0; count++; }

    if (count > 1) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0; // se para si hay conflicto
    }

    cmd_vel_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_north_, sub_south_, sub_east_, sub_west_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, bool> intrusions_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorNode>());
  rclcpp::shutdown();
  return 0;
}