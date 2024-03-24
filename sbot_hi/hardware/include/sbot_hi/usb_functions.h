// Comunicación
#include <iostream>
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions (escritura por usb)
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <poll.h>       // Monitorear archivo hasta q esten listos para ser leídos o haya transcurrido un tiempo máximo

#include "rclcpp/rclcpp.hpp"

int write_usb(int USB, const void *buf, size_t count) {
    return write(USB, buf, count);
}

void read_usb(int USB, unsigned char *buf, size_t count) {
  // struct pollfd fds[1];
  // fds[0].fd = USB;
  // fds[0].events = POLLIN; // El evento que "senso" es la llegada de un byte

  // // Esperar hasta que haya datos disponibles o hasta que hayan transcurrido 50 ms
  // int timeout = 5; // en milisegundos
  // int result = poll(fds, 1, timeout); // Llegó algo al USB o se terminó el tiempo
  // if (result == -1) {
  //   // Error al llamar a poll
  // } else if (result == 0) {
  //   RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
  //       "No llegó nada en %dms",timeout);
  // } else {
    // Leer el byte del puerto USB
    ssize_t bytes_read = read(USB, buf, count);
    if ((bytes_read == -1) || (bytes_read == 0)) {
        // Error al leer el byte
        RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Error al leer el dato: %ld",bytes_read);
      }
  // }
  return;
}


void read_usb_float(int USB, unsigned char (*datos)[4], int cantidad){
  // int count = 0;
  unsigned char byte = 0;

      // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
      // "Entrando a leer los datos!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  for(int dato = 0; dato < cantidad; dato++){
    for(int count = 0; count < 4; count++) {

      // Lectura del puerto
      ssize_t bytes_read = read(USB, &byte, 1);  // Leer un byte desde el puerto serie
      if ((bytes_read == -1) || (bytes_read == 0)) {
        // Error al leer el byte
        RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Error al leer el dato %d del USB %d",dato+1, USB);
        break;
      }

      datos[dato][count]=byte;
      // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
      // "Dato n° %d, byte %d: %d",dato, count, static_cast<int>(byte));
    }
  }
}

int conection_usb(int USB1, int USB2){
    /* Error Handling */
    if ( USB1 < 0 )
    {
    std::cout << "Error " << errno << " opening " << "/dev/ttyACM0" << ": " << strerror (errno) << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Error al recibir el usb1");
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
    std::cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Error al recibir el usb2");
    }
    // this->configuracion_port(USB);
    /* *** Configure Port *** */
    struct termios tty2;
    struct termios tty2_old;
    memset (&tty2, 0, sizeof tty2);

    /* Error Handling */
    if ( tcgetattr ( USB2, &tty2 ) != 0 ) {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
      return 0;
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
      return 0;
    }
    return 1;
}
