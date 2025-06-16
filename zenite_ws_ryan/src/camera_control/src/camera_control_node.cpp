#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "camera_control/camera_control.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
