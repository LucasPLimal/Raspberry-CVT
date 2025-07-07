#include <chrono>
#include <cmath>
#include <memory>
#include <iostream>
#include <vector>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>  // Mensagem para publicar velocidades

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
  ControlNode(float x_target = 5.0f, float y_target = 5.0f)
  : Node("control_node"),
    L(0.075f), R(0.02f),
    dt(0.01f), tempo_simulacao(10.0f),
    passos(static_cast<int>(tempo_simulacao / dt)),
    x(0.0f), y(0.0f), theta(0.0f),
    x_ref(x_target), y_ref(y_target),
    e_linear_ant(0.0f), e_angular_ant(0.0f),
    i_linear(0.0f), i_angular(0.0f),
    i(0),
    kc_angular(5.0f), Ti_angular(1000.0f),
    kc_linear(1.0f), Ti_linear(10000.0f),
    integral_limit_linear(5.0f), integral_limit_angular(5.0f)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("wheel_velocities", 10);

    timer = this->create_wall_timer(std::chrono::duration<float>(dt), std::bind(&ControlNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (i >= passos) {
      save_to_csv();
      print_final_state();
      rclcpp::shutdown();
      return;
    }

    float delta_d = std::hypot(x_ref - x, y_ref - y);
    if (std::abs(delta_d) <= 0.0001f) {
      save_to_csv();
      print_final_state();
      rclcpp::shutdown();
      return;
    }

    float theta_ref = std::atan2(y_ref - y, x_ref - x);
    float error_angular = theta_ref - theta;

    if (error_angular > M_PI_2)
      error_angular = M_PI_2;
    else if (error_angular < -M_PI_2)
      error_angular = -M_PI_2;

    float error_linear = std::hypot(y_ref - y, x_ref - x) * std::cos(error_angular);

    float p_linear = kc_linear * error_linear;
    i_linear += (kc_linear * dt * e_linear_ant) / Ti_linear;
    i_linear = std::clamp(i_linear, -integral_limit_linear, integral_limit_linear);
    float u_linear = p_linear + i_linear;
    e_linear_ant = error_linear;

    float p_angular = kc_angular * error_angular;
    i_angular += (kc_angular * dt * e_angular_ant) / Ti_angular;
    i_angular = std::clamp(i_angular, -integral_limit_angular, integral_limit_angular);
    float u_angular = p_angular + i_angular;
    e_angular_ant = error_angular;

    float v_direito = (2.0f * u_linear + u_angular * L) / (2.0f * R);
    float v_esquerdo = (2.0f * u_linear - u_angular * L) / (2.0f * R);

    float v = (v_direito + v_esquerdo) * R / 2.0f;
    float omega = (v_direito - v_esquerdo) * R / L;

    x += v * std::cos(theta) * dt;
    y += v * std::sin(theta) * dt;
    theta += omega * dt;

    // Guarda dados pra salvar depois
    trajetoria_x.push_back(x);
    trajetoria_y.push_back(y);
    trajetoria_theta.push_back(theta * 180.0f / M_PI);
    tempo.push_back(i * dt);
    vel_dir.push_back(v_direito);
    vel_esq.push_back(v_esquerdo);

    // Publica velocidades
    auto msg = geometry_msgs::msg::Vector3();
    msg.x = v_direito;
    msg.y = v_esquerdo;
    //msg.z = 0.0;
    publisher_->publish(msg);

    i++;
  }

  void save_to_csv()
  {
    std::ofstream file("trajetoria.csv");
    file << "tempo,x,y,theta,vel_dir,vel_esq\n";
    for (size_t j = 0; j < trajetoria_x.size(); j++) {
      file << tempo[j] << "," << trajetoria_x[j] << "," << trajetoria_y[j] << "," << trajetoria_theta[j] << "," << vel_dir[j] << "," << vel_esq[j] << "\n";
    }
    file.close();
    std::cout << "Dados salvos em trajetoria.csv\n";
  }

  void print_final_state() const
  {
    float theta_degrees = theta * 180.0f / M_PI;
    std::cout << "Simulacao finalizada.\n";
    std::cout << "Posicao final: x = " << x << ", y = " << y << ", theta = " << theta_degrees << " graus\n";
  }

  // Parâmetros robô e PID
  float L, R;
  float dt, tempo_simulacao;
  int passos;

  // Estado atual e referência
  float x, y, theta;
  float x_ref, y_ref;

  // PID estados
  float e_linear_ant, e_angular_ant;
  float i_linear, i_angular;

  // Constantes PID
  float kc_angular, Ti_angular;
  float kc_linear, Ti_linear;
  float integral_limit_linear, integral_limit_angular;

  // Contador
  int i;

  // Vetores de dados
  std::vector<float> trajetoria_x;
  std::vector<float> trajetoria_y;
  std::vector<float> trajetoria_theta;
  std::vector<float> tempo;
  std::vector<float> vel_dir;
  std::vector<float> vel_esq;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  float x_ref = 5.0f;
  float y_ref = 5.0f;

  if (argc >= 3) {
    x_ref = std::stof(argv[1]);
    y_ref = std::stof(argv[2]);
  }

  rclcpp::spin(std::make_shared<ControlNode>(x_ref, y_ref));
  rclcpp::shutdown();
  return 0;
}
