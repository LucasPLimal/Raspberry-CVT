#include <calibration_node/calibration_node.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibrationNode>());
    rclcpp::shutdown();
    return 0;
}
