#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <zenite_interfaces/srv/pixel_to_meter.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode() : Node("localization_node") {
        RCLCPP_INFO(this->get_logger(), "Localization Node iniciado.");

        // Subscriber da imagem do acquisition_node
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_frame", 10,
            std::bind(&LocalizationNode::image_callback, this, std::placeholders::_1));

        // Criação do serviço
        service_ = this->create_service<zenite_interfaces::srv::PixelToMeter>(
            "convert_pixel_to_meter",
            std::bind(&LocalizationNode::convert_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Janela para selecionar os pontos
        namedWindow("Selecao");
        setMouseCallback("Selecao", cliqueMouse, this);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Service<zenite_interfaces::srv::PixelToMeter>::SharedPtr service_;
    cv::Mat latest_image_;
    vector<Point2f> pontosImagem_;
    Mat H_;

    // -------------------- Subscriber da imagem --------------------
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            if (cv_ptr->image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Imagem recebida está vazia!");
                return;
            }
            latest_image_ = cv_ptr->image;
            processar_imagem();

            // Exemplo fixo de pixel a converter
            if (!H_.empty()) {
                Point2f exemplo_pixel(317, 213);
                vector<Point2f> entrada = {exemplo_pixel}, saida;
                perspectiveTransform(entrada, saida, H_);
                cout << "[Exemplo] Ponto na imagem: " << exemplo_pixel 
                     << " -> Ponto no chão (m): " << saida[0] << endl;
            }

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }

    // -------------------- Processa a imagem e desenha pontos --------------------
    void processar_imagem() {
        if (latest_image_.empty()) return;

        Mat temp;
        latest_image_.copyTo(temp);

        for (const auto &p : pontosImagem_)
            circle(temp, p, 5, Scalar(0, 0, 255), -1);

        imshow("Selecao", temp);
        waitKey(30);

        // Se já temos 4 pontos e H ainda não foi calculada
        if (pontosImagem_.size() == 4 && H_.empty()) {
            calcularHomografia();
        }
    }

    // -------------------- Callback do mouse --------------------
    static void cliqueMouse(int evento, int x, int y, int, void* userdata) {
        auto *self = reinterpret_cast<LocalizationNode*>(userdata);
        if (evento == EVENT_LBUTTONDOWN && self->pontosImagem_.size() < 4) {
            self->pontosImagem_.push_back(Point2f((float)x, (float)y));
            cout << "Ponto " << self->pontosImagem_.size() << ": (" << x << ", " << y << ")" << endl;
        }
    }

    // -------------------- Calcula a homografia --------------------
    void calcularHomografia() {
        vector<Point2f> pontosChao = {
            Point2f(0.0f, 0.0f),
            Point2f(2.75f, 0.0f),
            Point2f(2.75f, 2.25f),
            Point2f(0.0f, 2.25f)
        };

        H_ = findHomography(pontosImagem_, pontosChao);
        cout << "\nMatriz de Homografia calculada:\n" << H_ << endl;
    }

    // -------------------- Serviço de conversão pixel -> metro --------------------
    void convert_callback(
        const shared_ptr<zenite_interfaces::srv::PixelToMeter::Request> request,
        shared_ptr<zenite_interfaces::srv::PixelToMeter::Response> response)
    {
        if (H_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Homografia ainda não calculada!");
            response->x_m = 0.0f;
            response->y_m = 0.0f;
            return;
        }

        Point2f pontoImagem(request->x_pixel, request->y_pixel);
        vector<Point2f> entrada = {pontoImagem}, saida;
        perspectiveTransform(entrada, saida, H_);

        response->x_m = saida[0].x;
        response->y_m = saida[0].y;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<LocalizationNode>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
