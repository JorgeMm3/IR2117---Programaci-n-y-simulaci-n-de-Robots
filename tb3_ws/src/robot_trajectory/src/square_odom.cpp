#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;

double x, y, angle;

void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extraer la posición del robot
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    // Aquí podemos solo trabajar con la posición (x, y)
    // No necesitamos calcular la orientación, por lo que no usamos cuaterniones
    std::cout << "Position X: " << x << std::endl;
    std::cout << "Position Y: " << y << std::endl;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    // Crear el nodo
    auto node = rclcpp::Node::make_shared("robot_mover");
    
    // Crear el suscriptor para recibir mensajes de odometría
    auto subscription = node->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, topic_callback);

    // Crear el publicador para enviar comandos de movimiento (cmd_vel)
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Declarar parámetros para la velocidad y el tamaño del cuadrado
    node->declare_parameter("square_length", 1.0);
    node->declare_parameter("linear_speed", 0.1);
    node->declare_parameter("angular_speed", M_PI_2);
    
    // Obtener los parámetros declarados
    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    double square_length = node->get_parameter("square_length").get_parameter_value().get<double>();
    
    // Crear el mensaje de Twist para mover al robot
    geometry_msgs::msg::Twist message;
    
    // Establecer la tasa de loop
    rclcpp::WallRate loop_rate(10ms);

    // Ejecutar el movimiento en forma de cuadrado
    for (int j = 0; j < 4; j++) {
        
        // Avanzar en línea recta (lado del cuadrado)
        int i = 0, n = square_length / (0.01 * linear_speed);
        while (rclcpp::ok() && (i < n)) {
            message.linear.x = linear_speed;
            message.angular.z = 0.0;
            publisher->publish(message);
            rclcpp::spin_some(node);  // Ejecutar el ciclo de eventos
            loop_rate.sleep();
            i++;
        }

        // Girar 90 grados (un ángulo de 90° = M_PI/2 radianes)
        i = 0;
        n = M_PI_2 / (0.01 * angular_speed);
        while (rclcpp::ok() && (i < n)) {
            message.linear.x = 0.0;
            message.angular.z = angular_speed;
            publisher->publish(message);
            rclcpp::spin_some(node);  // Ejecutar el ciclo de eventos
            loop_rate.sleep();
            i++;
        }
    }

    // Detener el robot después de completar el cuadrado
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher->publish(message);
    
    // Finalizar el nodo
    rclcpp::shutdown();
    return 0;
}
