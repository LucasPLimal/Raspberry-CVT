#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "camera_control/camera_control.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterfaceNode>();

    cv::namedWindow("Controles", cv::WINDOW_NORMAL);

    cv::createTrackbar("Saturation", "Controles", nullptr, 255,
        [](int val, void* ptr){
            auto node = static_cast<InterfaceNode*>(ptr);
            node->send_params(-1, val/255.0f, -1);
        }, node.get());

    int saturation = node->get_control(V4L2_CID_SATURATION);
    cv::setTrackbarPos("Saturation", "Controles", saturation);

    cv::createTrackbar("Brilho", "Controles", nullptr, 255,
        [](int val, void* ptr){
            auto node = static_cast<InterfaceNode*>(ptr);
            node->send_params(val / 255.0f, -1, -1);
        }, node.get());

    int brightness = node->get_control(V4L2_CID_BRIGHTNESS);
    cv::setTrackbarPos("Brilho", "Controles", brightness);

    cv::createTrackbar("Contraste", "Controles", nullptr, 255,
        [](int val, void* ptr){
            auto node = static_cast<InterfaceNode*>(ptr);
            node->send_params(-1, -1, val / 255.0f);
        }, node.get());
    
    int contrast = node->get_control(V4L2_CID_CONTRAST);
    cv::setTrackbarPos("Contraste", "Controles", contrast);

    cv::VideoCapture cap(0);
    while (rclcpp::ok()) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        cv::imshow("Camera", frame);
        if (cv::waitKey(30) == 27) break;
    }

    rclcpp::shutdown();
    return 0;
}
