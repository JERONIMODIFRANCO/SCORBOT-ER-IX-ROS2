// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "/home/gaston/sbot/src/SCORBOT-ER-IX-ROS2/sbot_hi/hardware/include/sbot_hi/usb_functions.h"

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions (escritura por usb)
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

using std::placeholders::_1;

// Para utilizarlo, pegar "ros2 topic pub /topic std_msgs/msg/String 'data: "#"' "
// Con "#" reemplazado por el comando que se quiera enviar hacia el control de bajo nivel

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("Listener")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 

private:

  void topic_callback(const std_msgs::msg::String & msg) const
  {    
    /* Open File Descriptor */
    int USB2 = open( "/dev/ttyACM0", O_RDWR| O_NOCTTY );
    int USB1 = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );

    /* Error Handling */
    if ( USB1 < 0 )
    {
    std::cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << std::endl;
    }
    /* *** Configure Port *** */
    struct termios tty1;
    struct termios tty1_old;
    memset (&tty1, 0, sizeof tty1);

    /* Error Handling */
    if ( tcgetattr ( USB1, &tty1 ) != 0 ) {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    /* Save old tty parameters */
    tty1_old = tty1;

    /* Set Baud Rate */
    // cfsetospeed (&tty1, (speed_t)B9600);
    // cfsetispeed (&tty1, (speed_t)B9600);
    cfsetispeed(&tty1, B38400);
    cfsetospeed(&tty1, B38400); // Máximo BR que soporta la librería

    /* Setting other Port Stuff */
    tty1.c_cflag     &=  ~PARENB;            // Make 8n1
    tty1.c_cflag     &=  ~CSTOPB;
    tty1.c_cflag     &=  ~CSIZE;
    tty1.c_cflag     |=  CS8;

    tty1.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty1.c_cc[VMIN]   =  1;                  // read doesn't block
    tty1.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty1.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty1);

    /* Flush Port, then applies attributes */
    tcflush( USB1, TCIFLUSH );
    if ( tcsetattr ( USB1, TCSANOW, &tty1 ) != 0) {
      std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }


    if ( USB2 < 0 )
    {
    std::cout << "Error " << errno << " opening " << "/dev/ttyS2" << ": " << strerror (errno) << std::endl;
    }
    // this->configuracion_port(USB);
    /* *** Configure Port *** */
    struct termios tty2;
    struct termios tty2_old;
    memset (&tty2, 0, sizeof tty2);

    /* Error Handling */
    if ( tcgetattr ( USB2, &tty2 ) != 0 ) {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    /* Save old tty parameters */
    tty2_old = tty2;

    /* Set Baud Rate */
    // cfsetospeed (&tty, (speed_t)B9600);
    // cfsetispeed (&tty, (speed_t)B9600);
    cfsetispeed(&tty2, B38400);
    cfsetospeed(&tty2, B38400);

    /* Setting other Port Stuff */
    tty2.c_cflag     &=  ~PARENB;            // Make 8n1
    tty2.c_cflag     &=  ~CSTOPB;
    tty2.c_cflag     &=  ~CSIZE;
    tty2.c_cflag     |=  CS8;

    tty2.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty2.c_cc[VMIN]   =  1;                  // read doesn't block
    tty2.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty2.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty2);

    /* Flush Port, then applies attributes */
    tcflush( USB2, TCIFLUSH );
    if ( tcsetattr ( USB2, TCSANOW, &tty2 ) != 0) {
      std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }


    // --------------- Write -------------------- 

    // Envía un comando pre-establecido
    // unsigned char cmd[] = msg.data.c_str()[];
    // int n_written = 0, spot = 0;
    // do {
    //     n_written = write( USB1, &cmd[spot], 1 );
    //     n_written = write( USB2, &cmd[spot], 1 );
    //   spot += n_written;
    // } while ((msg.data.length()- spot) !=0 && n_written > 0); 

    // Envío el texto recibido por tópico
    int n_written = 0, spot = 0;
    do {
        n_written = write( USB1, &msg.data.c_str()[spot], 1 );
        n_written = write( USB2, &msg.data.c_str()[spot], 1 );
      spot += n_written;
    } while ((msg.data.length()- spot) !=0 && n_written > 0); 
  
    
    // ------------------- Leyendo el comando recibido --------------------

    unsigned char datos1[3][4];
    unsigned char datos2[3][4];
    unsigned char byte;

    // RCLCPP_INFO(rclcpp::get_logger("Listener"),
    //   "Comenzando a leer");
    if(msg.data.c_str()[0] == 'H'){
      // Leemos X bytes por separados
      read_usb(USB1, &byte, 1);
      datos1[0][0] = byte;
      read_usb(USB1, &byte, 1);
      datos1[1][0] = byte;
      read_usb(USB1, &byte, 1);
      datos1[2][0] = byte;
      read_usb(USB2, &byte, 1);
      datos2[0][0] = byte;
      read_usb(USB2, &byte, 1);
      datos2[1][0] = byte;
      read_usb(USB2, &byte, 1);
      datos2[2][0] = byte;
    }else{
      // Leemos X números de 4 bytes cada uno
      read_usb_float(USB1, datos1, 3);
      read_usb_float(USB2, datos2, 3);
    }

    // Otra opción es leer X bytes por separados
    // read_usb(USB1, &byte, 1);
    // datos1[0][3] = byte;
    // byte = 0;
    // read_usb(USB1, &byte, 1);
    // datos1[0][2] = byte;
    // byte = 0;
    // read_usb(USB1, &byte, 1);
    // datos1[0][1] = byte;
    // byte = 0;
    // read_usb(USB1, &byte, 1);
    // datos1[0][0] = byte;
    // byte = 0;
    // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //   "Bytes1 %d/%d/%d/%d", static_cast<int>(datos1[0][0]), static_cast<int>(datos1[0][1]),
    //     static_cast<int>(datos1[0][2]), static_cast<int>(datos1[0][3]));
    // read(USB1, &buffer[0], 1);
    // read(USB1, &buffer[1], 1);
    // read(USB1, &buffer[2], 1);
    // read(USB1, &buffer[3], 1);
    // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //   "Bytes2 %d/%d/%d/%d", static_cast<int>(buffer[0]), static_cast<int>(buffer[1]),
    //    static_cast<int>(buffer[2]), static_cast<int>(buffer[3]));
    // read(USB1, &buffer[0], 1);
    // read(USB1, &buffer[1], 1);
    // read(USB1, &buffer[2], 1);
    // read(USB1, &buffer[3], 1);
    // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //   "Bytes3 %d/%d/%d/%d", static_cast<int>(buffer[0]), static_cast<int>(buffer[1]),
    //    static_cast<int>(buffer[2]), static_cast<int>(buffer[3]));
    
    
    //------------------------ Transformación de la lectura -----------------
    float corriente_1, corriente_2, corriente_3, corriente_4, corriente_5, corriente_6;
    float intermedio;
    int HyT[3];
    double pos_1, pos_2, pos_3,pos_4,pos_5,pos_6;

    if(msg.data.c_str()[0] == 'C'){
        std::memcpy(&corriente_1, datos1[0], sizeof(float));
        std::memcpy(&corriente_2, datos1[1], sizeof(float));
        std::memcpy(&corriente_3, datos1[2], sizeof(float));
        std::memcpy(&corriente_4, datos2[0], sizeof(float));
        std::memcpy(&corriente_5, datos2[1], sizeof(float));
        std::memcpy(&corriente_6, datos2[2], sizeof(float));
        RCLCPP_INFO(rclcpp::get_logger("Listener"),
        "Corriente Leida: %f/%f/%f/%f/%f/%f"
          ,corriente_1,corriente_2,corriente_3,corriente_4,corriente_5,corriente_6);

      } else if(msg.data.c_str()[0] == 'P'){
        // Guardamos los ángulos (en grados) de cada junta en el estado (en radianes) correspondiente
        std::memcpy(&intermedio, datos1[0], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          pos_1 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        std::memcpy(&intermedio, datos1[1], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          pos_2 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        std::memcpy(&intermedio, datos1[2], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          pos_3 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        std::memcpy(&intermedio, datos2[0], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          pos_4 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        std::memcpy(&intermedio, datos2[1], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          pos_5 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        std::memcpy(&intermedio, datos2[2], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          pos_6 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        RCLCPP_INFO(rclcpp::get_logger("Listener"),
        "Posición Leida: %g/%g/%g/%g/%g/%g",pos_1,pos_2,pos_3,pos_4,pos_5,pos_6);
    } else{
        for(int i = 0; i < 3; i++){
          HyT[i] = datos1[i][0];
          HyT[i+3] = datos2[i][0];  
        }
        RCLCPP_INFO(rclcpp::get_logger("Listener"),
        "HS o Temp: %d/%d/%d/%d/%d/%d",HyT[0],HyT[1],HyT[2],HyT[3],HyT[4],HyT[5]);
      }  

    close(USB1);
    close(USB2);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
