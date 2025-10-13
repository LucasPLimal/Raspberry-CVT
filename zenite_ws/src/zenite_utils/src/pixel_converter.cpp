#include "zenite_utils/pixel_converter.hpp"
#include <iostream>

namespace zenite_utils
{

void PixelConverter::setReferencePoints(const std::vector<cv::Point2f>& pixel_points,
                                        const std::vector<cv::Point2f>& real_points)
{
    if (pixel_points.size() != 4 || real_points.size() != 4)
    {
        std::cerr << "[PixelConverter] Precisam ser 4 pontos de referência." << std::endl;
        return;
    }

    pixel_points_ = pixel_points;
    real_points_ = real_points;

    // Calcula homografia
    H_ = cv::findHomography(pixel_points_, real_points_);
    H_inv_ = H_.inv();

    std::cout << "[PixelConverter] Homografia calculada." << std::endl;
}

cv::Point2f PixelConverter::pixelToMeter(const cv::Point2f& pixel_point) const
{
    std::vector<cv::Point2f> src = {pixel_point}, dst;
    cv::perspectiveTransform(src, dst, H_);
    return dst[0];
}

cv::Point2f PixelConverter::meterToPixel(const cv::Point2f& real_point) const
{
    std::vector<cv::Point2f> src = {real_point}, dst;
    cv::perspectiveTransform(src, dst, H_inv_);
    return dst[0];
}

void PixelConverter::saveToYaml(const std::string& path) const
{
    YAML::Node node;
    node["pixel_points"] = pixel_points_;
    node["real_points"] = real_points_;

    std::ofstream fout(path);
    fout << node;
    fout.close();

    std::cout << "[PixelConverter] Dados salvos em " << path << std::endl;
}

void PixelConverter::loadFromYaml(const std::string& path)
{
    try
    {
        YAML::Node node = YAML::LoadFile(path);
        pixel_points_ = node["pixel_points"].as<std::vector<cv::Point2f>>();
        real_points_ = node["real_points"].as<std::vector<cv::Point2f>>();

        // Recalcula homografia ao carregar
        H_ = cv::findHomography(pixel_points_, real_points_);
        H_inv_ = H_.inv();

        std::cout << "[PixelConverter] Dados carregados de " << path << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[PixelConverter] Erro ao carregar YAML: " << e.what() << std::endl;
    }
}

}  // namespace zenite_utils
