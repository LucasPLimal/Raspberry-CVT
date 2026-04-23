#include <localization_node/localization_node.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationNode>();
    rclcpp::Rate rate(30);

    cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();
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
                RCLCPP_INFO(node->get_logger(), "Rastreamento inicializado.");
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


