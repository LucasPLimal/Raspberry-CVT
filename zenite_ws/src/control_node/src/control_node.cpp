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
                      //last_publish_time_(this->now()),
                      //cooldown_duration_(500ms),
    L(0.016f), R(0.03f),
    dt(0.1f),
    e_linear_ant(0.0f), e_angular_ant(0.0f),
    i_linear(0.0f), i_angular(0.0f),
    i(0),
    kc_angular(200000.0f), Ti_angular(9000000000.0f),
    kc_linear(5.0f), Ti_linear(10000.0f),
    integral_limit_linear(5.0f), integral_limit_angular(5.0f),
    x(0.0f), y(0.0f), theta(0.0f), // theta inicial fixo
    x_ref(0.0f), y_ref(0.0f),
    has_current_pos(false), has_desired_pos(false),
    att_dx(0.0f), att_dy(0.0f)
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

    RCLCPP_INFO(this->get_logger(), "ControlNode iniciado e aguardando posições...");
  }

private:
  // Posição atual
  void currentPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 2) {
      float new_x = static_cast<float>(msg->data[0]);
      float new_y = static_cast<float>(msg->data[1]);

      // Se tivermos uma posição anterior válida, estimamos o theta a partir
      // do deslocamento medido — isso corrige deriva e erros do modelo.
      if (has_prev_pos_) {
        float dx = new_x - prev_x_;
        float dy = new_y - prev_y_;
        float dist = std::hypot(dx, dy);

        att_dx = dx;
        att_dy = dy;
      
        // atualiza theta apenas se houve deslocamento significativo
        if (dist > 1e-4f) {
          theta = std::atan2(dy, dx);
        }
      }

      prev_x_ = new_x;
      prev_y_ = new_y;
      has_prev_pos_ = true;

      x = new_x;
      y = new_y;
      has_current_pos = true;
      RCLCPP_INFO(this->get_logger(), "Pos atual: (%.2f, %.2f), theta=%.1f°", x, y, theta * 180.0f / M_PI);
    }
  }

  // Posição desejada
  void desiredPosCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    x_ref = msg->x;
    y_ref = msg->y;
    has_desired_pos = true;
    RCLCPP_INFO(this->get_logger(), "Nova pos desejada: (%.2f, %.2f)", x_ref, y_ref);
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
  // PID linear (usar erro atual no integral e relação Ki = Kc / Ti)
  float p_linear = kc_linear * error_linear;
  float Ki_linear = (Ti_linear > 0.0f) ? (kc_linear / Ti_linear) : 0.0f;
  i_linear += Ki_linear * error_linear * dt;
  i_linear = std::clamp(i_linear, -integral_limit_linear, integral_limit_linear);
  float u_linear = p_linear + i_linear;
  e_linear_ant = error_linear;

  // PID angular
  float p_angular = kc_angular * error_angular;
  float Ki_angular = (Ti_angular > 0.0f) ? (kc_angular / Ti_angular) : 0.0f;
  i_angular += Ki_angular * error_angular * dt;
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

  // Normaliza theta para [-pi, pi]
  while (theta > M_PI) theta -= 2.0f * M_PI;
  while (theta < -M_PI) theta += 2.0f * M_PI;

    // Publica velocidades
    //auto now = this->now();

        // Só publica se passou do cooldown
        //if (now - last_publish_time_ < cooldown_duration_) {
            //return;
        //}
    
    auto msg = geometry_msgs::msg::Vector3();
    msg.x = v_direito;
    msg.y = v_esquerdo;
    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
      "x=%.2f y=%.2f | Dest=(%.2f,%.2f) | Vd=%.2f Ve=%.2f | θ=%.1f° | θ_ref=%.1f° | dx=%.2f | dy=%.2f",
      x, y, x_ref, y_ref, v_direito, v_esquerdo, theta * 180 / M_PI, theta_ref * 180 / M_PI, att_dx, att_dy);
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
  float att_dx, att_dy;

  // Histórico de posição para estimar theta a partir das medições
  float prev_x_ = 0.0f;
  float prev_y_ = 0.0f;
  bool has_prev_pos_ = false;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr desired_pos_sub_;

  //rclcpp::Time last_publish_time_;
  //rclcpp::Duration cooldown_duration_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}