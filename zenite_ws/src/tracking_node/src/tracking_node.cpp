#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "zenite_utils/pixel_converter.hpp"
using namespace std::chrono_literals;

class TrackingNode : public rclcpp::Node {
public:
    TrackingNode() : Node("tracking_node"), 
                     last_publish_time_(this->now()),
                     cooldown_duration_(500ms) // Cooldown definido
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_frame", 10,
            std::bind(&TrackingNode::imageCallback, this, std::placeholders::_1));

        initial_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position", 10);

        std::string yaml_path = this->declare_parameter<std::string>(
            "scale_yaml_path", "/tmp/scale.yaml");

        converter_.loadFromYaml(yaml_path);

        RCLCPP_INFO(this->get_logger(), "TrackingNode iniciado!");
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

    // ⏳ Controle de cooldown
    rclcpp::Time last_publish_time_;
    rclcpp::Duration cooldown_duration_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackingNode>();
    rclcpp::Rate rate(30);

    cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();
    cv::Rect2d roi;
    bool inicializado = false;

    std::vector<cv::Point> trajetoria;

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        if (!node->hasImage()) {
            rate.sleep();
            continue;
        }

        cv::Mat frame = node->getLatestImage();
        if (frame.empty()) continue;

        if (!inicializado) {
            roi = cv::selectROI("Selecione o carrinho", frame);
            if (roi.width > 0 && roi.height > 0) {
                tracker->init(frame, roi);
                inicializado = true;
                cv::destroyWindow("Selecione o carrinho");
                RCLCPP_INFO(node->get_logger(), "🎯 Rastreamento inicializado.");
            }
        } else {
            cv::Rect roi_atual;
            bool sucesso = tracker->update(frame, roi_atual);

            if (sucesso) {
                roi = roi_atual;

                cv::rectangle(frame, roi, cv::Scalar(0, 255, 0), 2);

                cv::Point2f centro(
                    roi.x + roi.width / 2.0,
                    roi.y + roi.height / 2.0);

                trajetoria.push_back(centro);
                for (size_t i = 1; i < trajetoria.size(); ++i)
                    cv::line(frame, trajetoria[i - 1], trajetoria[i], cv::Scalar(255, 0, 0), 2);

                cv::Point2f world_point = node->converter_.pixelToMeter(centro);

                // Publicação com cooldown funcionando
                node->publishPosition(world_point);

            } else {
                cv::putText(frame, "Falha no rastreamento", cv::Point(50, 50),
                            cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
            }
        }

        cv::imshow("Rastreamento do Carrinho", frame);
        if (cv::waitKey(30) == 27) break;

        rate.sleep();
    }

    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}


