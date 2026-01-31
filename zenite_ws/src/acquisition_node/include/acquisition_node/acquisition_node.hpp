#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "image_adjust_msgs/msg/image_params.hpp"
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

class AcquisitionNode : public rclcpp::Node {
public:
    AcquisitionNode() : Node("acquisition_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_frame", 10);

        params_sub_ = this->create_subscription<image_adjust_msgs::msg::ImageParams>(
            "/image_params", 10,
            std::bind(&AcquisitionNode::params_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30 FPS
            std::bind(&AcquisitionNode::publish_frame, this)
        );


        cap_.open(2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Não foi possível abrir a câmera!");
        }

        fd_ = open("/dev/video2", O_RDWR);
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao acessar /dev/video2");
        }

        else {
        // Desativa o Auto Exposure
        struct v4l2_control ctrl{};
        ctrl.id = V4L2_CID_EXPOSURE_AUTO;
        ctrl.value = V4L2_EXPOSURE_MANUAL;
        if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Erro ao desativar Auto Exposure");
        }
        }

    }

    ~AcquisitionNode() {
        if (fd_ != -1) {
            close(fd_);
        }
    }

    void params_callback(const image_adjust_msgs::msg::ImageParams::SharedPtr msg) {
        // Atualiza os parâmetros atuais
        hue_ = msg->hue;
        saturation_ = msg->saturation;
        brightness_ = msg->brightness;

        send_params();
    }

    void send_params() {
        if (fd_ == -1) return;
        if (brightness_ >= 0) set_control(V4L2_CID_BRIGHTNESS, brightness_ * 255);
        if (saturation_ >= 0) set_control(V4L2_CID_SATURATION, saturation_ * 255);
        if (hue_ >= 0) set_control(V4L2_CID_CONTRAST, hue_ * 255);
    }

    int get_control(__u32 id) {
        if (fd_ == -1) return -1;
        struct v4l2_control control {};
        control.id = id;
        if (ioctl(fd_, VIDIOC_G_CTRL, &control) == -1) return -1;
        return control.value;
    }
    float hue_ = 0.5f;
    float saturation_ = 0.5f;
    float brightness_ = 0.5f;

private:
    void publish_frame() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
    }

    void set_control(__u32 id, int value) {
        struct v4l2_control control {};
        control.id = id;
        control.value = value;
        ioctl(fd_, VIDIOC_S_CTRL, &control);
    }

    int fd_ = -1;
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<image_adjust_msgs::msg::ImageParams>::SharedPtr params_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
