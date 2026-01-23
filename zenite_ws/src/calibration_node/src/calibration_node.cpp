#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "zenite_utils/pixel_converter.hpp"

using std::placeholders::_1;

class CalibrationNode : public rclcpp::Node
{
public:
    CalibrationNode()
    : Node("localization_node")
    {
        // Subscrição da câmera
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_frame", 10, std::bind(&CalibrationNode::imageCallback, this, _1));

        // Parâmetro para caminho do YAML
        scale_yaml_path_ = this->declare_parameter<std::string>("scale_yaml_path", "/tmp/scale.yaml");

        // Cria janela e seta callback do mouse
        cv::namedWindow("Calibration - Defina os Pontos");
        cv::setMouseCallback("Calibration - Defina os Pontos", onMouse, this);

        RCLCPP_INFO(this->get_logger(), "CalibrationNode iniciado!");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Desenha pontos clicados
            for (const auto& p : image_points_)
                cv::circle(frame, p, 5, cv::Scalar(0, 0, 255), -1);

            // Desenha linhas conectando os pontos
            if (image_points_.size() > 1)
            {
                for (size_t i = 0; i < image_points_.size(); ++i)
                    cv::line(frame, image_points_[i], image_points_[(i+1) % image_points_.size()], cv::Scalar(255, 0, 0), 2);
            }

            cv::imshow("Calibration - Defina os Pontos", frame);
            cv::waitKey(1);

            // Quando os 4 pontos estão definidos e ainda não salvamos
            if (image_points_.size() == 4 && world_points_.size() == 4 && !saved_)
            {
                RCLCPP_INFO(this->get_logger(), "Calculando transformação e salvando escala...");

                converter_.setReferencePoints(image_points_, world_points_);
                converter_.saveToYaml(scale_yaml_path_);

                saved_ = true;
                RCLCPP_INFO(this->get_logger(), "Transformação salva em %s", scale_yaml_path_.c_str());
            }

            // Exemplo: converter um ponto usando PixelConverter
            if (saved_)
            {
                cv::Point2f pontoImagem(317, 217);  // pixel a ser convertido
                cv::Point2f pontoMundo = converter_.pixelToMeter(pontoImagem);

                RCLCPP_INFO(this->get_logger(), "Ponto na imagem: (%.1f, %.1f)", pontoImagem.x, pontoImagem.y);
                RCLCPP_INFO(this->get_logger(), "Ponto no chão (m): (%.2f, %.2f)", pontoMundo.x, pontoMundo.y);
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
        }
    }

    // Callback do mouse
    static void onMouse(int event, int x, int y, int, void* userdata)
    {
        auto* self = static_cast<CalibrationNode*>(userdata);

        if (event == cv::EVENT_LBUTTONDOWN && self->image_points_.size() < 4)
        {
            self->image_points_.push_back(cv::Point2f(x, y));
            RCLCPP_INFO(self->get_logger(), "Ponto de imagem (%d, %d) registrado", x, y);
        }

        // Quando 4 pontos clicados, define pontos do mundo automaticamente
        if (self->image_points_.size() == 4 && self->world_points_.empty())
        {
            self->world_points_ = {
                cv::Point2f(0.0f, 0.0f),
                cv::Point2f(2.75f, 0.0f),
                cv::Point2f(2.75f, 2.25f),
                cv::Point2f(0.0f, 2.25f)
            };
            RCLCPP_INFO(self->get_logger(), "Pontos do mundo definidos automaticamente (2.75 x 2.25 m).");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::string scale_yaml_path_;
    std::vector<cv::Point2f> image_points_;
    std::vector<cv::Point2f> world_points_;
    zenite_utils::PixelConverter converter_;
    bool saved_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibrationNode>());
    rclcpp::shutdown();
    return 0;
}
