#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "zenite_utils/pixel_converter.hpp"
using namespace std::chrono_literals;

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode() : Node("tracking_node"), 
                     last_publish_time_(this->now()),
                     cooldown_duration_(50ms) // Cooldown definido
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_frame", 10,
            std::bind(&LocalizationNode::imageCallback, this, std::placeholders::_1));

        initial_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/current_position", 10);

        std::string yaml_path = this->declare_parameter<std::string>(
            "scale_yaml_path", "/tmp/scale.yaml");

        converter_.loadFromYaml(yaml_path);

        RCLCPP_INFO(this->get_logger(), "Localization_Node iniciado!");
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            if (!cv_ptr->image.empty())
                latest_image_ = cv_ptr->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }

    cv::Mat getLatestImage() const { return latest_image_; }
    bool hasImage() const { return !latest_image_.empty(); }

    void publishPosition(const cv::Point2f& world_point)
    {
        auto now = this->now();

        // Só publica se passou do cooldown
        if (now - last_publish_time_ < cooldown_duration_) {
            return;
        }

        last_publish_time_ = now;

        std_msgs::msg::Float64MultiArray msg;
        msg.data = { world_point.x, world_point.y };
        initial_pos_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
            "Posicao enviada: (%.3f, %.3f) m",
            world_point.x, world_point.y);
    }

    zenite_utils::PixelConverter converter_;

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr initial_pos_pub_;

    cv::Mat latest_image_;

    rclcpp::Time last_publish_time_;
    rclcpp::Duration cooldown_duration_;
};