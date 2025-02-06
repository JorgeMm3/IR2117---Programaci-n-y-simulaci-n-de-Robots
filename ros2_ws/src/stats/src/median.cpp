#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int message_count = 0;
int sum = 0;
std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    sum += msg->data;  // Con esto sumo el valor del mensaje
    message_count++;    // E incremento el contador
    
    std_msgs::msg::Int32 out_msg;
    if (message_count > 0) {
        out_msg.data = sum / message_count;  // Calculo el promedio
    } else {
        out_msg.data = 0;  // Si no se recibe ningun mensaje, el promedio es 0
    }
    
    std::cout << "Suma: " << sum << " Contador: " << message_count << std::endl; // Comprovación
    publisher->publish(out_msg);  // Publicar el promedio
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mean_calculator_node");

    // Aqui me suscribo al nodo "number"
    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "number", 10, topic_callback);

    // Con esto publico en el nodo "mean"
    publisher = node->create_publisher<std_msgs::msg::Int32>("mean", 10);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}