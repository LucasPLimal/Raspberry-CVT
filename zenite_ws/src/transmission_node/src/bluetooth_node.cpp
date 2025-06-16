#include "transmission_node/bluetooth_node.h"

BluetoothNode::BluetoothNode() : Node("bluetooth_node") 
{
  subscription_ = this->create_subscription<transmission_node::msg::CustomData>(
                  "custom_topic", 10,
  std::bind(&BluetoothNode::messageCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Bluetooth Node has been started.");

  /*scan all active bluetooths*/
  //findActiveBluetoothDevice();
   
  /*seta endereços dos robôs*/ 

  btAction.setBluetoothAddr("10:23:09:01:36:17");
  btAction.setBluetoothAddr("98:D3:31:90:0C:FC");
  btAction.setBluetoothAddr("00:23:09:01:36:17");  //ESP32
  //btAction.setBluetoothAddr("00:15:83:35:75:A2"); 
  //btAction.setBluetoothAddr("20:16:09:18:98:39");
	
  /*identifica bluetooths proximos*/
  //btAction.findActiveBluetoothDevice();
	
  /*faz a conecção com os bluetooths with RFCOMM protocol*/
  //btAction.initBluetoothDevices(btAction.getNumBTDevices());
  btAction.initBluetoothDevices(3);
}

uint8_t BluetoothNode::pwmToByte(double pwm){
  pwm=fabs(pwm);
  if(pwm>=1.0)
    pwm=0.999999999;
  uint8_t byte = MAX_PWM_ARDUINO*pwm; //255 é o valor máximo de um byte para o arduino
  return byte; 
}

bool BluetoothNode::transmission(){
  static uint8_t dados[LEN_BUFFER];
  
  for(int i=0;i<3;i++)
  {
    dados[0]=0;
    // o sétimo bit do primeiro byte é o sentido do motor left 
    // 1= para frente , 0= para trás.
    if(me[i].left>= 0.0){
      dados[0]=0x02;
    }
    // o oitavo bit do primeiro byte é o sentido do motor right 
    // 1= para frente , 0= para trás.
    if(me[i].right>= 0.0){
      dados[0]+=0x01;
    }
    // o segundo byte é a velocidade do motor left
    dados[1]=pwmToByte(me[i].left);	
    // o terceiro byte é a velocidade do motor right
    dados[2]=pwmToByte(me[i].right);
    cout<<"dados  = "<<(int)dados[0] << ' ' <<(int)dados[1]<< ' ' <<(int)dados[2]<<endl;
    btAction.sendBluetoothMessage(i,dados);
  }
  return false;
}

void BluetoothNode::messageCallback(const transmission_node::msg::CustomData::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received: id=%d, vel_motor_right=%.2f, vel_motor_left=%.2f", msg->id, 
              msg->vel_motor_right, msg->vel_motor_left);
  int id = msg->id;
  int left = msg->vel_motor_left;
  int right = msg->vel_motor_right;
  if(id ==0 || id == 1 || id == 2){
    if(left >= -1.0 && left <= 1.0 && right >= -1.0 && right <= 1.0)
    {
          setVels(id,left,right);
          transmission();
    }
    else
    {
          cout << "Velocidades inválidas!";
    }
  }
  else
  {
    cout << "id inválido";
    
  }                  
  cout << "\n\n";  
}


int main(int argc, char **argv) {
  //ROS2
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BluetoothNode>());
  rclcpp::shutdown();
  // Resto do código
  return 0;
}
