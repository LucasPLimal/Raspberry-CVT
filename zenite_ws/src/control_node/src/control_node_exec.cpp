#include "control_node/control_node.h"

ControlNode::ControlNode() : Node("control_node") 
{
  publisher_ = this->create_publisher<control_node::msg::CustomData>("custom_topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&ControlNode::publishMessage, this));
  RCLCPP_INFO(this->get_logger(), "Control Node has been started.");
}

void ControlNode::publishMessage() {
  auto message = control_node::msg::CustomData();
  cout<<" Digite o id do robô"<<endl;
  cin>>id; cout<<endl;
  message.id = id;//generateRandomInt(0, 255);
  cout<<" Digite o vel do motor right "<<endl;
  cin>>right; cout<<endl;
  message.vel_motor_right = right;//generateRandomFloat(-1.0, 1.0);
  cout<<" Digite o vel do motor left "<<endl;
  cin>>left; cout<<endl;
  message.vel_motor_left = left;//generateRandomFloat(-1.0, 1.0);

  publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Published: id=%d, vel_motor_right=%.2f, vel_motor_left=%.2f", message.id, message.vel_motor_right, message.vel_motor_left);
  
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
