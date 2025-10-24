#include <chrono>
#include <cmath>
#include <memory>
#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control_node"),
    L(0.075f), R(0.02f),
    dt(0.01f),
    e_linear_ant(0.0f), e_angular_ant(0.0f),
    i_linear(0.0f), i_angular(0.0f),
    i(0),
    kc_angular(5.0f), Ti_angular(1000.0f),
    kc_linear(1.0f), Ti_linear(10000.0f),
    integral_limit_linear(5.0f), integral_limit_angular(5.0f),
    x(0.0f), y(0.0f), theta(0.0f), // theta inicial fixo
    x_ref(0.0f), y_ref(0.0f),
    has_current_pos(false), has_desired_pos(false)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("wheel_velocities", 10);

    current_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/position", 10,
      std::bind(&ControlNode::currentPosCallback, this, std::placeholders::_1));

    desired_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/desired_position", 10,
      std::bind(&ControlNode::desiredPosCallback, this, std::placeholders::_1));

    timer = this->create_wall_timer(
      std::chrono::duration<float>(dt),
      std::bind(&ControlNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "✅ ControlNode iniciado e aguardando posições...");
  }

private:
  // Posição atual
  void currentPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 2) {
      x = msg->data[0];
      y = msg->data[1];
      has_current_pos = true;
      RCLCPP_INFO(this->get_logger(), "📍 Pos atual: (%.2f, %.2f), theta=%.1f°", x, y, theta * 180 / M_PI);
    }
  }

  // Posição desejada
  void desiredPosCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    x_ref = msg->x;
    y_ref = msg->y;
    has_desired_pos = true;
    RCLCPP_INFO(this->get_logger(), "🎯 Nova pos desejada: (%.2f, %.2f)", x_ref, y_ref);
  }

  void timer_callback()
  {
    if (!has_current_pos || !has_desired_pos)
      return;

    float delta_d = std::hypot(x_ref - x, y_ref - y);
    if (delta_d <= 0.001f)
      return;

    float theta_ref = std::atan2(y_ref - y, x_ref - x);
    float error_angular = theta_ref - theta;

    // Normaliza o erro angular para [-pi, pi]
    while (error_angular > M_PI) error_angular -= 2 * M_PI;
    while (error_angular < -M_PI) error_angular += 2 * M_PI;

    float error_linear = delta_d * std::cos(error_angular);

    // PID linear
    float p_linear = kc_linear * error_linear;
    i_linear += (kc_linear * dt * e_linear_ant) / Ti_linear;
    i_linear = std::clamp(i_linear, -integral_limit_linear, integral_limit_linear);
    float u_linear = p_linear + i_linear;
    e_linear_ant = error_linear;

    // PID angular
    float p_angular = kc_angular * error_angular;
    i_angular += (kc_angular * dt * e_angular_ant) / Ti_angular;
    i_angular = std::clamp(i_angular, -integral_limit_angular, integral_limit_angular);
    float u_angular = p_angular + i_angular;
    e_angular_ant = error_angular;

    // Calcula velocidades das rodas
    float v_direito = (2.0f * u_linear + u_angular * L) / (2.0f * R);
    float v_esquerdo = (2.0f * u_linear - u_angular * L) / (2.0f * R);

    // Atualiza orientação (theta) pelo modelo diferencial
    float v = (v_direito + v_esquerdo) * R / 2.0f;
    float omega = (v_direito - v_esquerdo) * R / L;
    theta += omega * dt;

    // Mantém theta no intervalo [0, 2π)
    if (theta > 2 * M_PI) theta -= 2 * M_PI;
    else if (theta < 0) theta += 2 * M_PI;

    // Publica velocidades
    auto msg = geometry_msgs::msg::Vector3();
    msg.x = v_direito;
    msg.y = v_esquerdo;
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
      "🚗 x=%.2f y=%.2f | Dest=(%.2f,%.2f) | θ=%.1f° | Vd=%.2f Ve=%.2f",
      x, y, x_ref, y_ref, theta * 180 / M_PI, v_direito, v_esquerdo);
  }
// 342.0, 284.0
  // Variáveis principais
  float L, R, dt;
  float kc_angular, Ti_angular;
  float kc_linear, Ti_linear;
  float integral_limit_linear, integral_limit_angular;
  float x, y, theta;
  float x_ref, y_ref;
  float e_linear_ant, e_angular_ant;
  float i_linear, i_angular;
  int i;
  bool has_current_pos, has_desired_pos;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_pos_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
