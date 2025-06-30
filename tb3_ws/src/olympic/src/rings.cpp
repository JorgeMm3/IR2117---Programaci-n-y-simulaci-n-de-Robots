#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <string>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

// Función plantilla para esperar un servicio
template<typename ServiceT>
int waitForService(const std::shared_ptr<rclcpp::Client<ServiceT>>& client)
{
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service.");
      return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Service not available, waiting again...");
  }
  return 0;
}

// Función para esperar respuesta del servicio SetPen
void waitForResponse(
  const rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture& result,
  const std::shared_ptr<rclcpp::Node>& node,
  const std::string& message)
{
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "%s", (message + " set correctly.").c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s", ("Failed to call service " + message).c_str());
  }
}

// Función para esperar respuesta del servicio TeleportAbsolute
void waitForResponse(
  const rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture& result,
  const std::shared_ptr<rclcpp::Node>& node,
  const std::string& message)
{
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "%s", (message + " set correctly.").c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s", ("Failed to call service " + message).c_str());
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rings");

  node->declare_parameter("radius", 1.0);
  double radius = node->get_parameter("radius").get_parameter_value().get<double>();

  // Posiciones
  double altura_alta = 5.5 + radius / 2;
  double altura_baja = 5.5 - radius / 2;
  double distancia_centros_x = 1.083 * radius;

  double x_1 = 5.5 - distancia_centros_x * 2;
  double x_2 = 5.5;
  double x_3 = 5.5 + distancia_centros_x * 2;
  double x_4 = 5.5 - distancia_centros_x;
  double x_5 = 5.5 + distancia_centros_x;

  std::vector<std::vector<double>> colors = {
    {0, 0, 255},    // Blue
    {0, 0, 0},      // Black
    {255, 0, 0},    // Red
    {255, 255, 0},  // Yellow
    {0, 255, 0}     // Green
  };

  std::vector<std::vector<double>> positions = {
    {x_1, altura_alta},
    {x_2, altura_alta},
    {x_3, altura_alta},
    {x_4, altura_baja},
    {x_5, altura_baja}
  };

  // Publisher y clientes
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
  geometry_msgs::msg::Twist message;
  rclcpp::WallRate loop_rate(10ms);

  auto client_teleport_absolute = node->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
  auto client_setpen = node->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");

  auto request_setpen = std::make_shared<turtlesim::srv::SetPen::Request>();
  auto request_teleport_absolute = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();

for (int r = 0; r < 5; r++)
{
  // Apagar pluma
  request_setpen->off = true;
  if (waitForService(client_setpen) == 1) return 1;
  waitForResponse(client_setpen->async_send_request(request_setpen).future.share(), node, "Pen OFF");

  // Teletransportar
  request_teleport_absolute->x = positions[r][0];
  request_teleport_absolute->y = positions[r][1];
  if (waitForService(client_teleport_absolute) == 1) return 1;
  waitForResponse(client_teleport_absolute->async_send_request(request_teleport_absolute).future.share(), node, "Teleport Absolute");

  // Encender pluma con color y ancho
  request_setpen->off = false;
  request_setpen->r = colors[r][0];
  request_setpen->g = colors[r][1];
  request_setpen->b = colors[r][2];
  request_setpen->width = 2;
  if (waitForService(client_setpen) == 1) return 1;
  waitForResponse(client_setpen->async_send_request(request_setpen).future.share(), node, "Pen ON");

  // Dibujar círculo
  double loop_wait = 0.01;
  double linear_vel = 2.0;
  double distance = 2 * M_PI * radius;
  double linear_iterations = distance / (loop_wait * linear_vel);

  for (int i = 0; rclcpp::ok() && i < linear_iterations; i++) {
    message.linear.x = linear_vel;
    message.angular.z = linear_vel / radius;
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  // Detener movimiento
  message.linear.x = 0.0;
  message.angular.z = 0.0;
  publisher->publish(message);
}


  RCLCPP_INFO(node->get_logger(), "All rings drawn. Shutting down.");
  rclcpp::shutdown();
  return 0;
}
