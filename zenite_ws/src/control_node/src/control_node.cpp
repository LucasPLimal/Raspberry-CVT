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
    L(0.16f),           // Distância entre rodas (ajustar conforme robô real)
    R(0.03f),            // Raio da roda (ajustar conforme robô real)
    dt(0.01f),           // Período de controle (10ms)
    kc_angular(5.0f),    // Ganho proporcional angular
    Ti_angular(1000.0f), // Tempo integral angular
    kc_linear(1.0f),     // Ganho proporcional linear
    Ti_linear(10000.0f), // Tempo integral linear
    integral_limit_linear(5.0f),
    integral_limit_angular(5.0f),
    x(0.0f), y(0.0f), theta(0.0f),
    x_ref(0.0f), y_ref(0.0f),
    e_linear_ant(0.0f), e_angular_ant(0.0f),
    i_linear(0.0f), i_angular(0.0f),
    has_current_pos(false), has_desired_pos(false),
    is_moving(false)
  {
    // Publicador para velocidades das rodas
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("wheel_velocities", 10);

    // Subscritor para posição atual (do sistema de visão/sensores)
    current_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/current_position", 10,
      std::bind(&ControlNode::currentPosCallback, this, std::placeholders::_1));

    // Subscritor para posição desejada
    desired_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/desired_position", 10,
      std::bind(&ControlNode::desiredPosCallback, this, std::placeholders::_1));

    // Timer para loop de controle
    timer_ = this->create_wall_timer(
      std::chrono::duration<float>(dt),
      std::bind(&ControlNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "ControlNode iniciado");
    RCLCPP_INFO(this->get_logger(), "Aguardando posição atual e posição desejada...");
  }

private:
  // Callback para posição atual (de sensores externos)
  void currentPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 2) {
      // Espera-se: [x, y, theta] ou [x, y]
      float new_x = static_cast<float>(msg->data[0]);
      float new_y = static_cast<float>(msg->data[1]);
      float new_theta = theta; // mantém atual se não houver

      if (msg->data.size() >= 3) {
        // Se fornecerem theta diretamente
        new_theta = static_cast<float>(msg->data[2]);
      } else if (has_current_pos) {
        // Estima theta a partir do deslocamento
        float dx = new_x - x;
        float dy = new_y - y;
        float dist = std::hypot(dx, dy);
        
        if (dist > 0.001f) { // Movimento significativo
          new_theta = std::atan2(dy, dx);
        }
      }

      x = new_x;
      y = new_y;
      theta = new_theta;
      has_current_pos = true;

      RCLCPP_DEBUG(this->get_logger(), "Pos atual: (%.3f, %.3f), θ=%.2f°", 
                   x, y, theta * 180.0f / M_PI);
    }
  }

  // Callback para posição desejada
  void desiredPosCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    x_ref = msg->x;
    y_ref = msg->y;
    has_desired_pos = true;
    
    // Reseta integradores quando muda referência
    i_linear = 0.0f;
    i_angular = 0.0f;
    is_moving = true;
    
    RCLCPP_INFO(this->get_logger(), "Nova referência: (%.3f, %.3f)", x_ref, y_ref);
  }

  // Loop principal de controle
  void controlLoop()
  {
    if (!has_current_pos || !has_desired_pos || !is_moving) {
      return;
    }

    // Calcula distância até o alvo
    float delta_x = x_ref - x;
    float delta_y = y_ref - y;
    float distance = std::hypot(delta_x, delta_y);

    // Critério de parada
    if (distance < 0.01f) { // 1 cm de tolerância
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "Alvo alcançado!");
      is_moving = false;
      return;
    }

    // Ângulo desejado (em direção ao alvo)
    float theta_ref = std::atan2(delta_y, delta_x);
    
    // Erro angular (normalizado entre -π e π)
    float error_angular = theta_ref - theta;
    while (error_angular > M_PI) error_angular -= 2.0f * M_PI;
    while (error_angular < -M_PI) error_angular += 2.0f * M_PI;

    // Limita erro angular para ±90° (evita manobras bruscas)
    const float MAX_ANGULAR_ERROR = M_PI_2;
    error_angular = std::clamp(error_angular, -MAX_ANGULAR_ERROR, MAX_ANGULAR_ERROR);

    // Erro linear (projeção da distância na direção atual)
    float error_linear = distance * std::cos(error_angular);
    
    // **CONTROLE PID - Linear**
    float p_linear = kc_linear * error_linear;
    
    // Termo integral (Ki = Kc/Ti)
    float Ki_linear = (Ti_linear > 0.0f) ? (kc_linear / Ti_linear) : 0.0f;
    i_linear += Ki_linear * error_linear * dt;
    i_linear = std::clamp(i_linear, -integral_limit_linear, integral_limit_linear);
    
    // Sinal de controle linear
    float u_linear = p_linear + i_linear;
    e_linear_ant = error_linear;

    // **CONTROLE PID - Angular**
    float p_angular = kc_angular * error_angular;
    
    // Termo integral
    float Ki_angular = (Ti_angular > 0.0f) ? (kc_angular / Ti_angular) : 0.0f;
    i_angular += Ki_angular * error_angular * dt;
    i_angular = std::clamp(i_angular, -integral_limit_angular, integral_limit_angular);
    
    // Sinal de controle angular
    float u_angular = p_angular + i_angular;
    e_angular_ant = error_angular;

    // **CINEMÁTICA DIFERENCIAL**
    // v = velocidade linear, ω = velocidade angular
    float v = u_linear;
    float omega = u_angular;

    // Conversão para velocidades das rodas
    float v_direito = (2.0f * v + omega * L) / (2.0f * R);
    float v_esquerdo = (2.0f * v - omega * L) / (2.0f * R);

    // Publica velocidades
    publishWheelVelocities(v_direito, v_esquerdo);

    // Log
    RCLCPP_INFO(this->get_logger(), 
      "Pos: (%.3f, %.3f) | Ref: (%.3f, %.3f) | Dist: %.3f | "
      "θ: %.1f° | θ_ref: %.1f° | Vd: %.2f | Ve: %.2f",
      x, y, x_ref, y_ref, distance,
      theta * 180.0f / M_PI, theta_ref * 180.0f / M_PI,
      v_direito, v_esquerdo);
  }

  void publishWheelVelocities(float v_right, float v_left)
  {
    auto msg = geometry_msgs::msg::Vector3();
    msg.x = v_right;
    msg.y = v_left;
    msg.z = 0.0; // pode ser usado para flags se necessário
    publisher_->publish(msg);
  }

  void stopRobot()
  {
    publishWheelVelocities(0.0f, 0.0f);
    RCLCPP_INFO(this->get_logger(), "Robô parado");
  }

  // Parâmetros do robô
  const float L;      // Distância entre rodas (m)
  const float R;      // Raio da roda (m)
  const float dt;     // Período de controle (s)

  // Ganhos PID
  const float kc_angular, Ti_angular;
  const float kc_linear, Ti_linear;
  const float integral_limit_linear, integral_limit_angular;

  // Estado atual
  float x, y, theta;          // Posição e orientação atuais
  float x_ref, y_ref;         // Posição desejada

  // Estados do PID
  float e_linear_ant, e_angular_ant;
  float i_linear, i_angular;

  // Flags
  bool has_current_pos;
  bool has_desired_pos;
  bool is_moving;

  // ROS2
  rclcpp::TimerBase::SharedPtr timer_;
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

