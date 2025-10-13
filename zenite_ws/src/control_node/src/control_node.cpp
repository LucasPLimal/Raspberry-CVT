#include <chrono>
#include <cmath>
#include <memory>
#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp> // Para receber posições
#include <std_msgs/msg/float64_multi_array.hpp> // Posição atual (tracking)

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
    x(0.0f), y(0.0f), theta(0.0f),
    x_ref(0.0f), y_ref(0.0f),
    has_current_pos(false), has_desired_pos(false)
  {
    // Publisher das velocidades das rodas
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("wheel_velocities", 10);

    // Subscreve na posição atual do tracking
    current_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/position", 10,
      std::bind(&ControlNode::currentPosCallback, this, std::placeholders::_1));

    // Subscreve na posição desejada do interface
    desired_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/desired_position", 10,
      std::bind(&ControlNode::desiredPosCallback, this, std::placeholders::_1));

    // Timer para o loop de controle
    timer = this->create_wall_timer(
      std::chrono::duration<float>(dt),
      std::bind(&ControlNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "✅ ControlNode iniciado e aguardando posições...");
  }

private:
  // Callback da posição atual (tracking)
  void currentPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 2) {
      x = msg->data[0];
      y = msg->data[1];
      has_current_pos = true;
      RCLCPP_INFO(this->get_logger(), "📍 Posição atual recebida: x = %.3f, y = %.3f", x, y);
    }
  }

  // Callback da posição desejada (interface)
  void desiredPosCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    x_ref = msg->x;
    y_ref = msg->y;
    has_desired_pos = true;
    RCLCPP_INFO(this->get_logger(), "🎯 Nova posição desejada: x_ref = %.3f, y_ref = %.3f", x_ref, y_ref);
  }

  // Loop principal
  void timer_callback()
  {
    if (!has_current_pos || !has_desired_pos) {
      // Espera até ter as duas posições
      return;
    }

    float delta_d = std::hypot(x_ref - x, y_ref - y);
    if (delta_d <= 0.001f) {
      RCLCPP_INFO(this->get_logger(), "✅ Chegou ao destino: (%.3f, %.3f)", x, y);
      return;
    }

    // Calcula direção e erros
    float theta_ref = std::atan2(y_ref - y, x_ref - x);
    float error_angular = theta_ref - theta;

    // Limita erro angular
    if (error_angular > M_PI_2) error_angular = M_PI_2;
    else if (error_angular < -M_PI_2) error_angular = -M_PI_2;

    float error_linear = std::hypot(y_ref - y, x_ref - x) * std::cos(error_angular);

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

    // Cálculo das velocidades das rodas
    float v_direito = (2.0f * u_linear + u_angular * L) / (2.0f * R);
    float v_esquerdo = (2.0f * u_linear - u_angular * L) / (2.0f * R);

    // Publica velocidades
    auto msg = geometry_msgs::msg::Vector3();
    msg.x = v_direito;
    msg.y = v_esquerdo;
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
      "🚗 Controle -> PosAtual(%.2f, %.2f), Dest(%.2f, %.2f), VelD=%.2f, VelE=%.2f",
      x, y, x_ref, y_ref, v_direito, v_esquerdo);

    i++;
  }

  // Parâmetros do robô e PID
  float L, R, dt;
  float kc_angular, Ti_angular;
  float kc_linear, Ti_linear;
  float integral_limit_linear, integral_limit_angular;

  // Estado e referência
  float x, y, theta;
  float x_ref, y_ref;

  // PID estados
  float e_linear_ant, e_angular_ant;
  float i_linear, i_angular;

  int i;

  // Flags
  bool has_current_pos;
  bool has_desired_pos;

  // ROS objetos
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
