#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "image_adjust_msgs/msg/image_params.hpp"

class InterfaceNode : public rclcpp::Node {
public:
    InterfaceNode() : Node("interface_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_frame", 10,
            std::bind(&InterfaceNode::image_callback, this, std::placeholders::_1));

        params_pub_ = this->create_publisher<image_adjust_msgs::msg::ImageParams>(
            "image_params", 10);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            if (cv_ptr->image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Imagem recebida está vazia!");
                return;
            }

            latest_image_ = cv_ptr->image;

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }

    void send_params(float hue, float saturation, float brightness) {
        auto msg = image_adjust_msgs::msg::ImageParams();
        msg.hue = hue;
        msg.saturation = saturation;
        msg.brightness = brightness;
        params_pub_->publish(msg);
    }

    cv::Mat latest_image_;

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<image_adjust_msgs::msg::ImageParams>::SharedPtr params_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterfaceNode>();

    cv::namedWindow("Camera");
    cv::namedWindow("Controles");

    int hue = 32;
    int saturation = 32;
    int brightness = 128;

    cv::createTrackbar("Hue", "Controles", &hue, 255);
    cv::createTrackbar("Saturation", "Controles", &saturation, 255);
    cv::createTrackbar("Brightness", "Controles", &brightness, 255);

    rclcpp::Rate rate(30); // 30 Hz

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // Atualiza janela da câmera
        if (!node->latest_image_.empty()) {
            cv::imshow("Camera", node->latest_image_);
        }

        // Envia os parâmetros dos sliders
        node->send_params(hue / 255.0f, saturation / 255.0f, brightness / 255.0f);

        // Tecla ESC para sair
        int key = cv::waitKey(1);
        if (key == 27) break;

        rate.sleep();
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
