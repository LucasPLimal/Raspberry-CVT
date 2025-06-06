/* 
 * Cpp Code File: mainBluetooth.cpp
 * Very small Soccer - Bluetooth Team 2015.
 * Descrição: Programa principal de comunicação que envia dados para cada robô (N robôs) atraves do bluetooth de uma workstatioN (computador).
 * Decription: Main program of communication that  send data to each robots (N robots) by bluetooth from one workstation (PC). 
 * 
 * autor: valbercesar@gmail.com
 * Natal, 17/09/2015, 16:00h
 */

#include <string>
#include "transmission_node/bluetoothAction.h"
#include <sys/socket.h> /*communications stuffs by socket - also used by bluetooth */ 
#include <bluetooth/bluetooth.h> /*lib for bluetooth comm*/
#include <bluetooth/hci.h> /*lib for bluetooth comm*/
#include <bluetooth/hci_lib.h> /*lib for bluetooth comm*/
#include <bluetooth/rfcomm.h> /*lib for bluetooth comm*/
#include <bluetooth/l2cap.h>/*lib for bluetooth l2cap  */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
using namespace std;

/*constructor*/
BluetoothAction::BluetoothAction() {
    /*default values*/
    LEN_BUFFER = 3;
    NUM_BT_DEVICES = 3; 
    LEN_ADDR_BT_DEVICES = 18;
}

/*destructor*/
BluetoothAction::~BluetoothAction() { }

/*send message*/
bool BluetoothAction::sendBluetoothMessage(const unsigned int id,
					   const unsigned char message[]) {
  status[id] = write(sock[id], message, LEN_BUFFER );
  if (status[id] != LEN_BUFFER) {
    return true;
  }
  return false;
}

/*closeBluetooths modules*/
void BluetoothAction::closeBluetooths(int bt) {
  for(int t=0;t<bt;t++)
      close(sock[t]);
}

int BluetoothAction::initBluetoothById(int idBT) {
    /*convert string_STL to char* to be used on 'str2ba()' function*/
    const char* dest_char = dest[idBT].c_str();
  
    /* init robot socket 001*/
    sock[idBT] = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba(dest_char, &addr.rc_bdaddr );

    // connect to server
    status[idBT] = connect(sock[idBT], (struct sockaddr *)&addr, sizeof(addr));
     
    if( status[idBT] == 0 ) {
      cout<<"Connected to..."<<dest_char<<"\n\n";        
    }

    if( status[idBT] < 0 ) {
      cerr<<"Error to connect to.."<<dest_char<<endl<<endl;
      //return 0;      
    }
    return 0;
}
/*make connections to bluetooth setted before ::BluetoothAction()*/
int BluetoothAction::initBluetoothDevices(int bt) {
   
  for(int n = 0;n < bt; n++){
      /*convert string_STL to char* to be used on 'str2ba()' function*/
      const char* dest_char = dest[n].c_str(); 
  
      /* init robot socket 001 - 00N(nesimo)*/
      sock[n] = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
     
      // set the connection parameters (who to connect to)
      addr.rc_family = AF_BLUETOOTH;
      addr.rc_channel = (uint8_t) 1;
      str2ba( dest_char, &addr.rc_bdaddr );

      // connect to server
      status[n] = connect(sock[n], (struct sockaddr *)&addr, sizeof(addr));
     
      if( status[n] == 0 ) {
        cout<<"Connected to..."<<dest_char<<endl<<endl;        
      }

      if( status[n] < 0 ) {
        cerr<<"Error to connect to.."<<dest_char<<endl<<endl;
      // return 0;      
      }
      
   }//end_for
   return 0;
}

int BluetoothAction::getStatus0(){
      return status[0];
}

int BluetoothAction::getStatus1(){
      return status[1];
}

int BluetoothAction::getStatus2(){
      return status[2];
}

int BluetoothAction::initBluetoothDevicesL2CAP(int bt) {
  for(int n = 0;n < bt; n++){
      /*convert string_STL to char* to be used on 'str2ba()' function*/
      const char* dest_char = dest[n].c_str(); 
  
      /* init robot socket 001 - 00N(nesimo)*/
      sock[n] = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);

      // set the connection parameters (who to connect to)
       addr2.l2_family = AF_BLUETOOTH;
       addr2.l2_psm = htobs(0x1001); // bind socket to port 0x0 
       str2ba( dest_char, &addr2.l2_bdaddr );

      // connect to server
      status[n] = connect(sock[n], (struct sockaddr *)&addr2, sizeof(addr2));
     
      if( status[n] == 0 ) {
        cout<<"[L2CAP] Connected to..."<<dest_char<<endl<<endl;        
      }

      if( status[n] < 0 ) {
        cerr<<"[L2CAP] Error to connect to.."<<dest_char<<endl<<endl;
      // return 0;      
      }
   }//end_for
   return 0;
}

int BluetoothAction::findActiveBluetoothDevice(void) {
  inquiry_info *ii = NULL;
    int max_rsp, num_rsp;
    int dev_id, sock, len, flags;
    int i;
    char addr[19] = { 0 };
    char name[248] = { 0 };
    //vector<string> tmp_addrs;
    
    cout<<"[Seeking for active bluetooth, please wait]..\n";

    dev_id = hci_get_route(NULL);
    sock = hci_open_dev( dev_id );

    if (dev_id < 0 || sock < 0) {
        cerr<<"error: opening socket"<<endl;
        exit(1);
    }

    len  = 8;
    max_rsp = 255;
    flags = IREQ_CACHE_FLUSH;
    ii = (inquiry_info*)malloc(max_rsp * sizeof(inquiry_info));
    
    num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
    if( num_rsp < 0 ) perror("hci_inquiry");

    cout<<"["<< num_rsp <<"] Bluetooth device(s) founded..\n\n";

    for (i = 0; i < num_rsp; i++) {
        ba2str(&(ii+i)->bdaddr, addr);
        memset(name, 0, sizeof(name));
        if (hci_read_remote_name(sock, &(ii+i)->bdaddr, sizeof(name), 
            name, 0) < 0)
        strcpy(name, "[unknown]");
        cout<<""<<addr<<"  "<<name<<"\n";
	//tmp_addrs.push_back(new string(addr));
    }
    free( ii );
    close( sock );
    cout<<"\nEnd of '::findActiveBluetoothDevice()'..\n"<<endl;

    return 0;
  //return tmp_addrs;
} //end of method

//getters
vector<string> BluetoothAction::getDest(void) {return dest;}
int BluetoothAction::getLenBuffer(void) { return LEN_BUFFER;}
int BluetoothAction::getNumBTDevices(void) { return NUM_BT_DEVICES;}
int BluetoothAction::getLenAddrBTDevices(void) {return LEN_ADDR_BT_DEVICES;}
string BluetoothAction::getBluetoothAddr(int pos) {return dest[pos];}
vector<int> BluetoothAction::getStatus() { return tmp_status; }

//setters
void BluetoothAction::setLenBuffer(int newLenBuffer) { LEN_BUFFER = newLenBuffer; }
void BluetoothAction::setNumBTDevices(int newNumBT) { NUM_BT_DEVICES = newNumBT; }
void BluetoothAction::setLenAddrBTDevices(int newLenAddr) { LEN_ADDR_BT_DEVICES = newLenAddr; }
void BluetoothAction::setBluetoothAddr(string newBTAddr) {dest.push_back(newBTAddr);}
