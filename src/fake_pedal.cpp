#include <stdio.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

int main(int argc, char *argv[])
{
    // ROS stuff
    // https://roboticsbackend.com/write-minimal-ros2-cpp-node/
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("fake_infinity_pedal");
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher = node->create_publisher<std_msgs::msg::Bool>("infinity_pedal/triggered", 10);

    // ROS params
    node->declare_parameter("state", false);
    bool STATE = node->get_parameter("state").as_bool();

    
    while (rclcpp::ok()){
        // ROS publishing
        auto msg = std_msgs::msg::Bool();
        msg.data = STATE;
        publisher->publish(msg);

        // use rclcpp to wait for 0.05 s (20 Hz, 50 ms)
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    rclcpp::shutdown();

    return 0;
}

