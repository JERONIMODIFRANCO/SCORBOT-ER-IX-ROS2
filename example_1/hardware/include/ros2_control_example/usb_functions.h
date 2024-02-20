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
  // int timeout = 50; // en milisegundos
  // int result = poll(fds, 1, timeout); // Llegó algo al USB o se terminó el tiempo
  // if (result == -1) {
  //   // Error al llamar a poll
  // } else if (result == 0) {
  //   // Se se agota el tiempo dejo el byte le asigno '255' = Error
  //   *buf = 255;
  // } else {
    // Leer el byte del puerto USB
    ssize_t bytes_read = read(USB, buf, count);
    if ((bytes_read == -1) || (bytes_read == 0)) {
        // Error al leer el byte
        RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Error al leer el dato: %ld",bytes_read);
      }
  // }
  return;
}


void read_usb_float(int USB, unsigned char (*datos)[4], int cantidad){
  // int count = 0;
  unsigned char byte = 0;

      // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      // "Entrando a leer los datos!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  for(int dato = 0; dato < cantidad; dato++){
    for(int count = 0; count < 4; count++) {

      // Lectura del puerto
      ssize_t bytes_read = read(USB, &byte, 1);  // Leer un byte desde el puerto serie
      if ((bytes_read == -1) || (bytes_read == 0)) {
        // Error al leer el byte
        RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Error al leer el dato: %ld",bytes_read);
      }

      datos[dato][count]=byte;
      // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      // "Dato n° %d, byte %d: %d",dato, count, static_cast<int>(byte));
    }
  }
}

int conection_usb(int USB){
    /* Error Handling */
    if ( USB < 0 )
    {
    std::cout << "Error " << errno << " opening " << "/dev/ttyACM0" << ": " << strerror (errno) << std::endl;
      return 0;
    }
    // this->configuracion_port(USB);
    /* *** Configure Port *** */
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( USB, &tty ) != 0 ) {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
      return 0;
    }

    /* Save old tty parameters */
    tty_old = tty;

    /* Set Baud Rate */
    // cfsetospeed (&tty, (speed_t)B9600);
    // cfsetispeed (&tty, (speed_t)B9600);
    cfsetispeed(&tty, B38400);
    cfsetospeed(&tty, B38400); // Creo que es el máximo BR que soporta la librería

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
      return 0;
    }
    return 1;
}