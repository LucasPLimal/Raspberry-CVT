#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <algorithm>

using boost::asio::serial_port_base;

class TransmissionNode : public rclcpp::Node {
public:
  TransmissionNode()
  : Node("transmission_node"),
    io_(),
    serial_(io_),
  {
    try {
      
      serial_.open("/dev/rfcomm0"); // Código para associar mac a porta rfcomm: sudo rfcomm bind /dev/rfcomm0 00:23:09:01:36:17 (Parnadilo)
      serial_.set_option(serial_port_base::baud_rate(9600));
      serial_.set_option(serial_port_base::character_size(8));
      serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
      serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

      RCLCPP_INFO(this->get_logger(), "Porta serial /dev/rfcomm0 aberta com sucesso!");
    }
    catch (std::exception &e) {
      RCLCPP_FATAL(this->get_logger(), "Falha ao abrir porta serial: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "wheel_velocities", 10,
      std::bind(&TransmissionNode::vel_callback, this, std::placeholders::_1));

    // Executor async do Boost.Asio
    io_thread_ = std::thread([this]() { io_.run(); });
  }

  ~TransmissionNode() {
    if (serial_.is_open())
      serial_.close();

    io_.stop();
    if (io_thread_.joinable())
      io_thread_.join();
  }

private:
  void vel_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    float v_dir = msg->x;
    float v_esq = msg->y;

    uint8_t direcao = 0;

    if (v_dir >= 0) direcao |= 0b00000001;
    if (v_esq >= 0) direcao |= 0b00000010;

    uint8_t pwm_esq = static_cast<uint8_t>(
       std::abs(v_esq) * 255.0f 

      );

    uint8_t pwm_dir = static_cast<uint8_t>( 
      std::abs(v_dir) * 255.0f 
    );

    uint8_t pacote[3] = {direcao, pwm_esq, pwm_dir};

    try {
      boost::asio::write(serial_, boost::asio::buffer(pacote, 3));
      RCLCPP_INFO(this->get_logger(),
        "Enviado → DIR=%d | PWM_ESQ=%d | PWM_DIR=%d",
        direcao, pwm_esq, pwm_dir);
    }
    catch (std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Erro ao enviar pacote serial: %s", e.what());
    }
  }

  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  std::thread io_thread_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_;
  float vmax_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransmissionNode>());
  rclcpp::shutdown();
  return 0;
}
