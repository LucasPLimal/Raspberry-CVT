
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>
#include <vector>
#include <string>



class TrackingNode : public rclcpp::Node {
public:
    TrackingNode() : Node("tracking_node"), latest_image_ready_(false) {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_frame", 10,
            std::bind(&TrackingNode::image_callback, this, std::placeholders::_1));

        init_value_pub_ = this->create_publisher<std_msgs::msg::String>(
            "init_value", 10);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            if (cv_ptr->image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Imagem recebida está vazia!");
                return;
            }
            latest_image_ = cv_ptr->image;
            latest_image_ready_ = true;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }

    void send_init_value(const std::string &value) {
        auto msg = std_msgs::msg::String();
        msg.data = value;
        init_value_pub_->publish(msg);
    }

    cv::Mat get_latest_image() {
        latest_image_ready_ = false;
        return latest_image_.clone();
    }

    bool has_image() const {
        return latest_image_ready_;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr init_value_pub_;
    cv::Mat latest_image_;
    bool latest_image_ready_;
};


using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackingNode>();
    rclcpp::Rate rate(30); // 30 Hz
    Ptr<Tracker> tracker = TrackerKCF::create(); // Ou outro rastreador
    Rect2d roi;
    bool inicializado = false;
    vector<Point> trajetoria;

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (!node->has_image()) {
            rate.sleep();
            continue;
        }
        Mat frame = node->get_latest_image();
        if (frame.empty()) {
            cout << "Erro ao carregar " << endl;
            continue;
        }
        if (!inicializado) {
            roi = selectROI("Selecione o carrinho", frame);
            if (roi.width > 0 && roi.height > 0) {
                tracker->init(frame, roi);
                inicializado = true;
                destroyWindow("Selecione o carrinho");
            }
        } else {
            Rect roi_int = roi; // Converte de Rect2d para Rect<int>
            bool sucesso = tracker->update(frame, roi_int);
            if (sucesso) {
                roi = roi_int; // Atualiza a ROI principal com a nova posição
                rectangle(frame, roi, Scalar(0, 255, 0), 2);
                Point centro(roi.x + roi.width / 2, roi.y + roi.height / 2);
                trajetoria.push_back(centro);
                for (size_t j = 1; j < trajetoria.size(); ++j)
                    line(frame, trajetoria[j - 1], trajetoria[j], Scalar(255, 0, 0), 2);
            } else {
                putText(frame, "Falha no rastreamento", Point(50, 50),
                        FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
            }
        }
        imshow("Rastreamento do Carrinho", frame);
        waitKey(30);
        rate.sleep();
    }
    rclcpp::shutdown();
    destroyAllWindows();
    return 0;
}

