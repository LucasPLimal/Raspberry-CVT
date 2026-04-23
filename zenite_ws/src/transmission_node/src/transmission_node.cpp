#include <transmission_node/transmission_node.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransmissionNode>());
  rclcpp::shutdown();
  return 0;
}
