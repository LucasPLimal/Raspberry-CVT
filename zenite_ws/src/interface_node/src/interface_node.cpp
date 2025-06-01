#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp> 
#include <opencv2/opencv.hpp>
#include "image_adjust_msgs/msg/image_params.hpp"

class InterfaceNode : public rclcpp::Node {
public:
    InterfaceNode() : Node("interface_node") {
        // Publicador de parâmetros de ajuste
        //params_pub_ = this->create_publisher<image_adjust_msgs::msg::ImageParams>(
            //"image_params", 10);
            

            
        // Assinante da imagem processada
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&InterfaceNode::image_callback, this, std::placeholders::_1));
    }

    // Callback executado ao receber a imagem processada
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // Converte ROS Image para cv::Mat
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        
        // Verifica se a imagem é válida
        if(cv_ptr->image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Imagem recebida está vazia!");
            return;
        }
        
        // Exibe a imagem
        cv::imshow("Imagem Processada", cv_ptr->image);
        cv::waitKey(1);
        
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Erro no OpenCV: %s", e.what());
    }
}
    // Método para enviar parâmetros
    void send_params(float hue, float saturation, float brightness) {
        auto msg = image_adjust_msgs::msg::ImageParams();
        msg.hue = hue;
        msg.saturation = saturation;
        msg.brightness = brightness;
        params_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<image_adjust_msgs::msg::ImageParams>::SharedPtr params_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterfaceNode>();
    
    // Cria janela com trackbars
    cv::namedWindow("Controles");
    cv::createTrackbar("Hue", "Controles", nullptr, 100, [](int val, void* node_ptr){
        auto node = static_cast<InterfaceNode*>(node_ptr);
        node->send_params(val/100.0f, -1, -1); // -1 mantém valor atual
    }, node.get());
    
    cv::createTrackbar("Saturation", "Controles", nullptr, 100, [](int val, void* node_ptr){
        auto node = static_cast<InterfaceNode*>(node_ptr);
        node->send_params(-1, val/100.0f, -1); // -1 mantém valor atual
    }, node.get());
    
    cv::createTrackbar("Brightness", "Controles", nullptr, 100, [](int val, void* node_ptr){
        auto node = static_cast<InterfaceNode*>(node_ptr);
        node->send_params(-1, -1, val/100.0f); // -1 mantém valor atual
    }, node.get());
    
    

    
    
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
