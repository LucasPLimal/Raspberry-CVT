#include <interface_node/interface_node.hpp>

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
