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

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions (escritura por usb)
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:

  void configuracion_port(int USB){
    /* *** Configure Port *** */
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( USB, &tty ) != 0 ) {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    /* Save old tty parameters */
    tty_old = tty;

    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B9600);
    cfsetispeed (&tty, (speed_t)B9600);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( USB, TCIFLUSH );
    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
      std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }
  }

  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    RCLCPP_INFO(this->get_logger(), "Sending it back");
    
    /* Open File Descriptor */
    int USB = open( "/dev/ttyACM0", O_RDWR| O_NOCTTY );

    /* Error Handling */
    if ( USB < 0 )
    {
    std::cout << "Error " << errno << " opening " << "/dev/ttyACM0" << ": " << strerror (errno) << std::endl;
    }
    // this->configuracion_port(USB);
    /* *** Configure Port *** */
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( USB, &tty ) != 0 ) {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    /* Save old tty parameters */
    tty_old = tty;

    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B9600);
    cfsetispeed (&tty, (speed_t)B9600);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( USB, TCIFLUSH );
    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
      std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }


    //Write
    // unsigned char cmd[] = "INIT \r";
    int n_written = 0,
    spot = 0;

    // Original
    // do {
    //     n_written = write( USB, &cmd[spot], 1 );
    //     spot += n_written;
    // } while (cmd[spot-1] != '\r' && n_written > 0);

    // Utilizando texto por tópico
    do {
        n_written = write( USB, &msg.data.c_str()[spot], 1 );
        spot += n_written;
    } while ((msg.data.length()-1) !=0 && n_written > 0); //Cambiarlo por length(msg.data.c_str())
  
    //Utilizando números del código
    // int mi_entero = 83;  // Entero a convertir
    // char mi_char = static_cast<char>(mi_entero);  // Convertir el entero a un carácter
    // const void* mi_void_ptr = static_cast<const void*>(&mi_char);  // Convertir el carácter a const void*
    // write(USB, mi_void_ptr, 1);  
    // std::cout << "Error " << mi_char << std::endl;

    // Transcribiendo string a número
    // int mi_entero = atoi(msg.data.c_str());  // Convierto el string en un entero
    // char mi_char = static_cast<char>(mi_entero);  // Convertir el entero a un carácter
    // const void* mi_void_ptr = static_cast<const void*>(&mi_char);  // Convertir el carácter a const void*
    // write(USB, mi_void_ptr, 1);  
    // std::cout << "Error " << mi_char << std::endl;

    std::cout << "Enviado. Leyendo por el puerto serie" << std::endl;
    // Leyendo el string y separandolo
    // int recivedChar;
    unsigned char byte;
    // unsigned char bytes[18];
    int count = 0;
    int dato = 0;

    unsigned char bytes[3][18];

    while (true) {

      // Lectura del puerto
      int n = read(USB, &byte, 1);  // Leer un byte desde el puerto serie
      if (n == -1) {
          std::cout << "Error en la recepción por USB del código" << std::endl;
      }
      // Hacer que cuando tenga que enviar varios bytes, si uno de ellos es 0 que no lo envíe
      // (salvo el último obviamente). De esta forma, podemos dividirlos en varios string utilizando
      // la barra como división y que no se generen demasiados
      // if(byte == '\0'){
      //   byte = '0';
      // }

      // Almacenaje de los datos
      // std::cout << byte << std::endl; 
      if (byte == '\n') { // Termina el envío de los datos
          bytes[dato][count] = '\0'; //Indico que termina el string
          break;  // Salir del bucle si se recibe un byte de nueva línea
      }else if(byte == '/'){ // '/' indica que terminó el dato anterior
        bytes[dato][count] = '\0'; //indico que terminó un dato (esto solo para trabajarlo con string, luego se quita)
        dato = dato + 1; // Paso al siguiente dato
        count = 0; // Comienzo la cuenta de bytes del nuevo dato desde 0 
      } else { // El byte pertenece a un número
        bytes[dato][count] = byte;
        count = count + 1;
        }
    }
    
    std::cout << bytes[0] << '/' << bytes[1] << '/' << bytes[2] << std::endl;
    std::cout << count << std::endl;
    

    float corriente_1, corriente_2, corriente_3;
    double pos_1, pos_2, pos_3;
    int intermedio;
    int HyT[3];

    if(msg.data.c_str()[0] == 'C'){
        std::memcpy(&corriente_1, bytes[0], sizeof(float));
        std::memcpy(&corriente_2, bytes[1], sizeof(float));
        std::memcpy(&corriente_3, bytes[2], sizeof(float));
        std::cout << "C: " << corriente_1 << '/' << corriente_2 << '/' << corriente_3 << std::endl;
    } else if(msg.data.c_str()[0] == 'P'){
        std::memcpy(&intermedio, bytes[0], sizeof(int));
        pos_1 = intermedio;
        std::memcpy(&intermedio, bytes[1], sizeof(int));
        pos_2 = intermedio;
        std::memcpy(&intermedio, bytes[2], sizeof(int));
        pos_3 = intermedio;
        std::cout << "P: " << pos_1 << '/' << pos_2 << '/' << pos_3 << std::endl;
    } else{
        for(int i = 0; i < 3; i++){
          if(bytes[i][0] == '\0'){HyT[i] = 0;}
          else {HyT[i] = 1;}
        }
        std::cout << "H/T: " << HyT[0] << '/' << HyT[1] << '/' << HyT[2] << std::endl;
    }
    
    
    // Limpiar la variable bytes
    std::memset(bytes, 0, sizeof(bytes));


    // unsigned char bytess[] = {0x64, 0x47, 0x73, 0x47, 0x84, 0x0};  // Definir los bytes
    // // float f;
    // // std::memcpy(&f, bytess, sizeof(float));
    // // std::cout << f << std::endl;
    // // unsigned char bytess[] = {0x64, '/', 0x73, '/', 0x125, '\0'};
    // std::string input(reinterpret_cast<char*>(bytess), sizeof(bytess));
    // std::cout << input << std::endl;
    // // std::string input = std::string::str(std::begin(bytes), std::end(bytes) - 1);
    // std::stringstream ss(input); // Se crea un stringstream llamado "ss" a partir del string "input"
    // std::string token;
    // char delimiter = '/';
    // std::vector<std::string> tokens; // Se define un vector de strings llamado "tokens"

    // // Se utiliza la función getline para extraer los tokens del stringstream "ss" separados 
    // // por el delimitador, y almacenarlos en el string "token" hasta que ss se termine 
    // // (o sea que getline de 0)
    // while (std::getline(ss, token, delimiter)) { 
    //     tokens.push_back(token); // Se agrega cada token al vector "tokens"
    // }

    // for (const auto& t : tokens) { // Se recorre el vector "tokens" y se imprime cada token en una línea separada
    //     std::cout << t << '1' << std::endl;
    // }
  

    close(USB);
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
