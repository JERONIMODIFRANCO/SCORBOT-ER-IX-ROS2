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
#include "/home/gaston/sbot/src/sbot_hi/example_1/hardware/include/ros2_control_example/usb_functions.h"

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
    // cfsetospeed (&tty, (speed_t)B9600);
    // cfsetispeed (&tty, (speed_t)B9600);
    cfsetispeed(&tty, B38400);
    cfsetospeed(&tty, B38400);


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
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    // RCLCPP_INFO(this->get_logger(), "Sending it back");
    
    /* Open File Descriptor */
    int USB2 = open( "/dev/ttyACM0", O_RDWR| O_NOCTTY );
    int USB1 = open( "/dev/ttyS2", O_RDWR| O_NOCTTY );

    /* Error Handling */
    if ( USB1 < 0 )
    {
    std::cout << "Error " << errno << " opening " << "/dev/ttyACM0" << ": " << strerror (errno) << std::endl;
    }
    // this->configuracion_port(USB);
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
    // cfsetospeed (&tty, (speed_t)B9600);
    // cfsetispeed (&tty, (speed_t)B9600);
    cfsetispeed(&tty1, B38400);
    cfsetospeed(&tty1, B38400); // Creo que es el máximo BR que soporta la librería

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
    cfsetospeed(&tty2, B38400); // Creo que es el máximo BR que soporta la librería

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


    //Write
    unsigned char cmd = msg.data.c_str()[0];
    int n_written = 0, spot = 0;

    // Comando flotante
    // double comando[] = {-15.8, 15.3, -35.3};
    // float comando_f[] = {static_cast<float>(comando[0]), static_cast<float>(comando[1]),static_cast<float>(comando[2])};
    // float intermedio[3];
    // unsigned char bytes_comando[3][4];
    // std::memcpy(bytes_comando[0], &comando_f[0], sizeof(comando_f[0]));
    // std::memcpy(bytes_comando[1], &comando_f[1], sizeof(comando_f[1]));
    // std::memcpy(bytes_comando[2], &comando_f[2], sizeof(comando_f[2]));
    // std::memcpy(&intermedio[0], bytes_comando[0], sizeof(comando_f[0]));
    // std::memcpy(&intermedio[1], bytes_comando[1], sizeof(comando_f[1]));
    // std::memcpy(&intermedio[2], bytes_comando[2], sizeof(comando_f[2]));
    // std::cout << "Transformando " << comando[0] << " en: " << intermedio[0] << std::endl;
    // std::cout << "Transformando " << comando[1] << " en: " << intermedio[1] << std::endl;
    // std::cout << "Transformando " << comando[2] << " en: " << intermedio[2] << std::endl;

    // n_written = write( USB, cmd, 1 );
    // for(int i = 0; i < 3; i++){
    //   spot = 3;
    //   do{
    //     n_written = write( USB, &bytes_comando[i][spot], 1 );
    //     std::cout << "Enviando comando " << i << ", byte: " << static_cast<int>(bytes_comando[i][spot]) << " spot: " << spot << std::endl;
    //     spot -= n_written;
    //     // std::cout << "Spot luego: " << spot << "condición " << ((4- spot) > 4 && n_written > 0)<< std::endl;
    //   }while ((4- spot) < 5 && n_written > 0);
    // }

    // Original
    // do {
    //     n_written = write( USB, &cmd[spot], 1 );
    //     spot += n_written;
    // } while (cmd[spot-1] != '9' && n_written > 0);
    // n_written = 0;
    // spot = 0;

    // Utilizando texto por tópico
    do {
      // if(msg.data.c_str()[spot] == '0'){
      //   cmd[spot] = 0;
      //   n_written = write( USB, cmd, 1 );
      // }else if(msg.data.c_str()[spot] == '1'){
      //   cmd[spot] = 1;
      //   n_written = write( USB, cmd, 1 );
      // }else{
        n_written = write( USB1, &msg.data.c_str()[spot], 1 );
        n_written = write( USB2, &msg.data.c_str()[spot], 1 );
      // }
      spot += n_written;
    } while ((msg.data.length()- spot) !=0 && n_written > 0); //Cambiarlo por length(msg.data.c_str())
  
    // std::cout << "Comando enviado" << std::endl;
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

    // std::cout << "Enviado. Leyendo por el puerto serie" << std::endl;
    // Leyendo el string y separandolo
    int recivedChar;
    // unsigned char bytes[18];
    int count = 0;
    // int dato = 0;
    unsigned char byte = 0;
    unsigned char bytes[3][4];
    unsigned char dato1[4],dato2[4],dato3[4];
    unsigned char datos1[3][4];
    unsigned char datos2[3][4];
    int bandera = 0;

    // unsigned char bytes[3][18];

    if(msg.data.c_str()[0] != 'R'){
      RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Comenzando a leer");
      read_usb_float(USB1, datos1, 3);
      read_usb_float(USB2, datos2, 3);
      





      // while (true) {

      //   // Lectura del puerto
      //   int n = read(USB, &byte, 1);  // Leer un byte desde el puerto serie
      //   if (n == -1) {
      //       std::cout << "Error en la recepción por USB del código" << std::endl;
      //   }
      //   // Hacer que cuando tenga que enviar varios bytes, si uno de ellos es 0 que no lo envíe
      //   // (salvo el último obviamente). De esta forma, podemos dividirlos en varios string utilizando
      //   // la barra como división y que no se generen demasiados
      //   // if(byte == '\0'){
      //   //   byte = '0';
      //   // }

      //   // Almacenaje de los datos
      //   // std::cout << byte << std::endl; 
      //   if (byte == '\n') { // Termina el envío de los datos
      //       bytes[dato][count] = '\0'; //Indico que termina el string
      //       std::cout << "Fin" << std::endl;
      //       break;  // Salir del bucle si se recibe un byte de nueva línea
      //   }else if(byte == '/'){ // '/' indica que terminó el dato anterior
      //     bytes[dato][count] = '\0'; //indico que terminó un dato (esto solo para trabajarlo con string, luego se quita)
      //     dato = dato + 1; // Paso al siguiente dato
      //     count = 0; // Comienzo la cuenta de bytes del nuevo dato desde 0 
      //     std::cout << "Dato"<< dato << ": "<< bytes[0] << std::endl;
      //   } else { // El byte pertenece a un número
      //     bytes[dato][count] = byte;
      //     count = count + 1;
      //     }
      // }

      

      // while (!bandera) {

      //   // Lectura del puerto
      //   read(USB, &byte, 1);  // Leer un byte desde el puerto serie
      //   if(count < 4){
      //     dato1[count]=byte;
      //     RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      //     "Dato n° 1: %d- count= %d",static_cast<int>(byte),count);
      //   }else if(count < 8){
      //     dato2[count-4]=byte;
      //      RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      //     "Dato n° 2: %d- count= %d",static_cast<int>(byte),count);
      //   }else if(count < 12){
      //     dato3[count-8]=byte;
      //      RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      //     "Dato n° 3: %d- count= %d",static_cast<int>(byte),count);
      //   }
      //   if(count > 10){
      //      RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      //     "Fuera- count= %d",count);
      //     bandera = 1;
      //   }
      //   count = count + 1;




      //   // El byte pertenece a un número
      //   // bytes[dato][count] = byte;
      //   // count = count + 1;

      //   // if (count > 3) { // Termina el envío de los datos
      //   //   if(dato == 2){
      //   //   RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      //   //   "Dato n° %d- count= %d (fin)",dato,count);
      //   //   bandera = 1;
      //   //     // bytes[dato][count] = '\0'; //Indico que termina el string
      //   //     // break;  // Salir del bucle si se recibe un byte de nueva línea
      //   //   } else{
      //   //   RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      //   //   "Dato n° %d- count= %d",dato,count);
      //   //   // bytes[dato][count] = '\0'; //indico que terminó un dato (esto solo para trabajarlo con string, luego se quita)
      //   //   dato = dato + 1; // Paso al siguiente dato
      //   //   count = 0; // Comienzo la cuenta de bytes del nuevo dato desde 0 
      //   //   }
      //   // }

      // }
    }
    
    // std::cout << bytes[0] << '/' << bytes[1] << '/' << bytes[2] << std::endl;
    // std::cout << count << std::endl;
    

    float corriente_1, corriente_2, corriente_3;
    float intermedio;
    int HyT[3];
    double pos_1, pos_2, pos_3,pos_4,pos_5,pos_6;

    if(msg.data.c_str()[0] == 'C'){
        std::memcpy(&corriente_1, datos1[0], sizeof(float));
        std::memcpy(&corriente_2, datos1[1], sizeof(float));
        std::memcpy(&corriente_3, datos1[2], sizeof(float));
        // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        // "Corriente Leida: %f/%f/%f",corriente_1,corriente_2,corriente_3);
        // std::cout << "C: " << corriente_1 << '/' << corriente_2 << '/' << corriente_3 << std::endl;
    } else if(msg.data.c_str()[0] == 'P'){
        // Guardamos los ángulos (en grados) de cada junta en el estado (en radianes) correspondiente
        std::memcpy(&intermedio, datos1[0], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[0] = intermedio*(3.1415/180); 
          pos_1 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Intermedio 1: %d",intermedio);
        std::memcpy(&intermedio, datos1[1], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[1] = intermedio*(3.1415/180); 
          pos_2 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        std::memcpy(&intermedio, datos1[2], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[2] = intermedio*(3.1415/180); 
          pos_3 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        // Guardamos los ángulos (en grados) de cada junta en el estado (en radianes) correspondiente
        std::memcpy(&intermedio, datos2[0], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[0] = intermedio*(3.1415/180); 
          pos_4 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Intermedio 1: %d",intermedio);
        std::memcpy(&intermedio, datos2[1], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[1] = intermedio*(3.1415/180); 
          pos_5 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        std::memcpy(&intermedio, datos2[2], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[2] = intermedio*(3.1415/180); 
          pos_6 = static_cast<double>(intermedio*(3.1415/180)); 
        }
        RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Posición Leida: %g/%g/%g/%g/%g/%g",pos_1,pos_2,pos_3,pos_4,pos_5,pos_6);
        // "Datos leidos: %g/%g",hw_states_[3],hw_states_[4]);
        // std::cout << "P: " << hw_states_[0] << '/' << hw_states_[1] << '/'
    } else{
        for(int i = 0; i < 3; i++){
          HyT[i] = static_cast<int>(datos1[i][0]);
        }
        RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Datos leidos: %d/%d/%d",HyT[0],HyT[1],HyT[2]);
        // std::cout << "H/T: " << HyT[0] << '/' << HyT[1] << '/' << HyT[2] << std::endl;
    }

    // static int cuenta = 0;
    // static float corriente_anterior = 0;

    // if(corriente_anterior == corriente_1){
    //   RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //   "Se repitió un valor");
    //   cuenta = 0;
    // }else {cuenta +=1;}

    // corriente_anterior = corriente_1;
    



    // float corriente_1, corriente_2, corriente_3;
    // double pos_1, pos_2, pos_3;
    // int intermedio;
    // int HyT[3];

    // if(msg.data.c_str()[0] == 'C'){
    //     std::memcpy(&corriente_1, dato1, sizeof(float));
    //     std::memcpy(&corriente_2, dato2, sizeof(float));
    //     std::memcpy(&corriente_3, dato3, sizeof(float));
    //     std::cout << "C: " << corriente_1 << '/' << corriente_2 << '/' << corriente_3 << std::endl;
    // } else if(msg.data.c_str()[0] == 'P'){
    //     std::memcpy(&intermedio, dato1, sizeof(int));
    //     pos_1 = intermedio*(3.1415/180);
    //     std::memcpy(&intermedio, dato2, sizeof(int));
    //     pos_2 = intermedio*(3.1415/180);
    //     std::memcpy(&intermedio, dato3, sizeof(int));
    //     pos_3 = intermedio*(3.1415/180);
    //     std::cout << "P: " << pos_1 << '/' << pos_2 << '/' << pos_3 << std::endl;
    // } else{
    //     for(int i = 0; i < 3; i++){
    //       if(bytes[i][0] == '\0'){HyT[i] = 0;}
    //       else {HyT[i] = 1;}
    //     }
    //     std::cout << "H/T: " << HyT[0] << '/' << HyT[1] << '/' << HyT[2] << std::endl;
    // }
    
    
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
