#include <rclcpp/rclcpp.hpp>
#include <control_node/msg/custom_data.hpp>
#include <iostream>

using namespace std::chrono_literals;
using namespace std;

class ControlNode : public rclcpp::Node {
public:
        ControlNode();
private:
    void publishMessage();
    rclcpp::Publisher<control_node::msg::CustomData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    uint id;
    float right;
    float left;
};


