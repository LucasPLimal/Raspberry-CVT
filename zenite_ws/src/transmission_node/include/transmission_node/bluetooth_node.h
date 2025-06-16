#include <iostream>
#include <math.h>
#include <transmission_node/bluetoothAction.h>
#include <time.h>
//ROS2
#include <rclcpp/rclcpp.hpp>
#include <transmission_node/msg/custom_data.hpp>

#define LEN_BUFFER 3

using namespace::std;

struct pwmRobo{
	double left,right;
};

class BluetoothNode : public rclcpp::Node {
public:
  // Constructor
  BluetoothNode();
  inline void setVels(int id_,int left_,int right_){ me[id_].left=left_; me[id_].right=right_;}
  bool transmission();
private:
  BluetoothAction btAction;
  const int MAX_PWM_ARDUINO = 256;
  pwmRobo me[3];

  uint8_t pwmToByte(double pwm);
  
  //ROS2
  void messageCallback(const transmission_node::msg::CustomData::SharedPtr msg); 
  rclcpp::Subscription<transmission_node::msg::CustomData>::SharedPtr subscription_;
};



