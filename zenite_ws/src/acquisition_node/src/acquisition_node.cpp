#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "acquisition_node/acquisition_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AcquisitionNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
