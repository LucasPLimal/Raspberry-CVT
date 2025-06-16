#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "image_adjust_msgs/msg/image_params.hpp"

class AcquisitionNode : public rclcpp::Node {
public:
    AcquisitionNode() : Node("acquisition_node") {
        // Inicializa a captura da webcam (0 = câmera padrão)
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a webcam!");
            rclcpp::shutdown();
        }

        // Configura a resolução (opcional)
        //cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        //cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        //cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0); 
        //cap_.set(cv::CAP_PROP_BRIGHTNESS, 64);
        // cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 95); 
        //cap_.set(cv::CAP_PROP_EXPOSURE, 50);

        // Publicador de imagens
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

        // Timer para captura contínua (30 FPS)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&AcquisitionNode::capture_and_publish, this));
    }

private:
    void capture_and_publish() {
        cv::Mat frame;
        cap_ >> frame; // Captura um frame
        
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Frame vazio recebido da câmera");
            return;
        }

        // Converte e publica
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        image_pub_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AcquisitionNode>());
    rclcpp::shutdown();
    return 0;
}
