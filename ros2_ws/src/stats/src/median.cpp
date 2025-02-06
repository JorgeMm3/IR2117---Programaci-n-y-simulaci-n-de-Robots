#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <vector>
#include <algorithm> 

std::vector<int> numbers;  // En este vector almaceno los números
std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    numbers.push_back(msg->data); // Añado el valor recibido al vector
    std::sort(numbers.begin(), numbers.end()); // Ordeno el vector
    
    // Muestro el vector de números recibido
    std::cout << "Vector de números ordenados: ";
    for (int num : numbers) {
        std::cout << num << " ";
    }
    std::cout << std::endl;

    // Calculo la mediana
    std_msgs::msg::Int32 out_msg;
    size_t size = numbers.size();

    if (size > 0) {
        if (size % 2 == 0) {
            // Si el número de elementos es par, calculo la mediana como el promedio de los dos elementos centrales
            out_msg.data = (numbers[size / 2 - 1] + numbers[size / 2]) / 2;
        } else {
            // Si el número de elementos es impar, la mediana es el valor central
            out_msg.data = numbers[size / 2];
        }
    } else {
        out_msg.data = 0;  // Si no se ha recibido ningún mensaje, la mediana es 0
    }

    publisher->publish(out_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("median");

    // Aqui me suscribo a "number"
    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "number", 10, topic_callback);

    // Con esto publico en "mean"
    publisher = node->create_publisher<std_msgs::msg::Int32>("median", 10);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}