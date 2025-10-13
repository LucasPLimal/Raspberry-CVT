#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "zenite_utils/pixel_converter.hpp"

// Inclua a mensagem personalizada
#include "image_adjust_msgs/msg/image_params.hpp"

class InterfaceNode : public rclcpp::Node {
public:
    InterfaceNode()
        : Node("interface_node")
    {
        // Subscription da câmera
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_frame", 10,
            std::bind(&InterfaceNode::imageCallback, this, std::placeholders::_1));

        // Publisher do ponto clicado
        desired_pos_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/desired_position", 10);

        // Publisher dos parâmetros de ajuste de imagem
        params_pub_ = this->create_publisher<image_adjust_msgs::msg::ImageParams>("image_params", 10);

        // Carrega a homografia
        std::string yaml_path = this->declare_parameter<std::string>("scale_yaml_path", "/tmp/scale.yaml");
        converter_.loadFromYaml(yaml_path);

        // Configura janela e callback do mouse
        cv::namedWindow("Camera");
        cv::namedWindow("Controles");
        cv::setMouseCallback("Camera", onMouse, this);

        // Cria sliders
        cv::createTrackbar("Hue", "Controles", &hue_, 255);
        cv::createTrackbar("Saturation", "Controles", &saturation_, 255);
        cv::createTrackbar("Brightness", "Controles", &brightness_, 255);

        RCLCPP_INFO(this->get_logger(), "InterfaceNode iniciado!");
    }

    // Getter para acessar a imagem no main()
    cv::Mat getLatestImage() const { return latest_image_; }

    // Função pública para publicar sliders
    void publishParams()
    {
        auto msg = image_adjust_msgs::msg::ImageParams();
        msg.hue = static_cast<float>(hue_) / 255.0f;
        msg.saturation = static_cast<float>(saturation_) / 255.0f;
        msg.brightness = static_cast<float>(brightness_) / 255.0f;
        params_pub_->publish(msg);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            if (!cv_ptr->image.empty())
                latest_image_ = cv_ptr->image;
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }

    static void onMouse(int event, int x, int y, int, void* userdata)
    {
        if (event != cv::EVENT_LBUTTONDOWN) return;

        auto* self = static_cast<InterfaceNode*>(userdata);
        if (self->latest_image_.empty()) return;

        cv::Point2f pixel_point(x, y);
        cv::Point2f world_point = self->converter_.pixelToMeter(pixel_point);

        // Publica ponto convertido
        geometry_msgs::msg::Point pt_msg;
        pt_msg.x = world_point.x;
        pt_msg.y = world_point.y;
        pt_msg.z = 0.0;
        self->desired_pos_pub_->publish(pt_msg);

        //RCLCPP_INFO(self->get_logger(), 
        //            "Clique na imagem: (%.1f, %.1f) -> Mundo: (%.2f, %.2f)",
        //            pixel_point.x, pixel_point.y, world_point.x, world_point.y);
    }

    cv::Mat latest_image_;
    zenite_utils::PixelConverter converter_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr desired_pos_pub_;
    rclcpp::Publisher<image_adjust_msgs::msg::ImageParams>::SharedPtr params_pub_;

    int hue_ = 32;
    int saturation_ = 32;
    int brightness_ = 128;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterfaceNode>();
    rclcpp::Rate rate(30); // 30 Hz

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // Atualiza janela da câmera
        cv::Mat img = node->getLatestImage();
        if (!img.empty())
            cv::imshow("Camera", img);

        // Publica parâmetros de sliders
        node->publishParams();

        int key = cv::waitKey(1);
        if (key == 27) break;  // ESC para sair

        rate.sleep();
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
