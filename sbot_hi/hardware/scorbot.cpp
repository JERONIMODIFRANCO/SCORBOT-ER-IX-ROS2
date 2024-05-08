// Copyright 2020 ros2_control Development Team
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

#include "sbot_hi/scorbot.hpp"
#include "sbot_hi/usb_functions.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


#include <cstdlib>
#include <fstream> // Incluir la cabecera para std::ofstream
#include "std_msgs/msg/string.hpp" // Incluir la cabecera del mensaje que deseas publicar



namespace sbot_hi
{
hardware_interface::CallbackReturn SbotPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  comando_viejo = 0;;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // SbotPositionOnlyHardware has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Init tópico de corriente
  node = rclcpp::Node::make_shared("pub_corriente"); // Inicialización de la variable node
  publisher = node->create_publisher<std_msgs::msg::String>("corriente_juntas", 10); // Inicialización de la variable publisher

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SbotPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SbotPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SbotPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SbotPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  // Starting the conection via SERIAL
  USB1 = open( "/dev/ttyUSB1", O_RDWR| O_NOCTTY );
  //  RCLCPP_INFO(
  //   rclcpp::get_logger("SbotPositionOnlyHardware"), "Activado 1 / %d",USB1);
  USB2 = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );
  //  RCLCPP_INFO(
  //   rclcpp::get_logger("SbotPositionOnlyHardware"), "Activado 2 / %d",USB2);
  // Finishing the conection via SERIAL

  unsigned char comando_lectura = 'I';
  ssize_t bytes_written;
  bytes_written = write_usb(USB1, &comando_lectura, 1);
  if(bytes_written == -1) {
      // Handle error
      RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
      "Error en el envío del comando de lectura");
  }
  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  
  bytes_written = write_usb(USB2, &comando_lectura, 1);
  if(bytes_written == -1) {
      // Handle error
      RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
      "Error en el envío del comando de lectura");
  }

  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SbotPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close(USB1);
  close(USB2);

  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SbotPositionOnlyHardware::read(
  const rclcpp::Time & tiempo, const rclcpp::Duration & /*periodo*/)
{
    static int primero = 1;
    static double star_time;
    double actual_time;
    if(primero){
      star_time = tiempo.seconds();
      primero = 0;
    }
    unsigned char comando_lectura = 'P';
    ssize_t bytes_written = write_usb(USB1, &comando_lectura, 1);
    if(bytes_written == -1) {
        // Handle error
        RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Error en el envío del comando de lectura");
    }
   bytes_written = write_usb(USB2, &comando_lectura, 1);
    if(bytes_written == -1) {
        // Handle error
        RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Error en el envío del comando de lectura");
    }

    unsigned char datos1[3][4], datos2[3][4];
    read_usb_float(USB1, datos1, 3);
    read_usb_float(USB2, datos2, 3);
    

    float intermedio;
    if(comando_lectura == 'P'){
      // Guardamos los ángulos (en grados) de cada junta en el estado (en radianes) correspondiente
      std::memcpy(&intermedio, datos1[0], sizeof(float));
      if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
        hw_states_[0] = static_cast<double>(-intermedio*(3.1415/180)); 
      }
      std::memcpy(&intermedio, datos1[1], sizeof(float));
      if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
        hw_states_[1] = static_cast<double>(intermedio*(3.1415/180)); 
      }
      std::memcpy(&intermedio, datos1[2], sizeof(float));
      if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
        hw_states_[2] = static_cast<double>(intermedio*(3.1415/180)); 
      }
      std::memcpy(&intermedio, datos2[0], sizeof(float));
      if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
        hw_states_[3] = static_cast<double>(-intermedio*(3.1415/180)); 
      }
      std::memcpy(&intermedio, datos2[1], sizeof(float));
      if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
        hw_states_[4] = static_cast<double>(intermedio*(3.1415/180)); 
      }
      std::memcpy(&intermedio, datos2[2], sizeof(float));
      if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
        hw_states_[5] = hw_commands_[5];
      }
      // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
      // "Posición Leida: %g/%g/%g/%g/%g/%g",hw_states_[0],hw_states_[1],hw_states_[2],hw_states_[3],hw_states_[4],hw_states_[5]);
    } 

    // Opción de leer y almacenar las corrientes
    // *****************************************
    comando_lectura = 'C';
    bytes_written = write_usb(USB1, &comando_lectura, 1);
    if(bytes_written == -1) {
        // Handle error
        RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Error en el envío del comando de lectura");
    }
   bytes_written = write_usb(USB2, &comando_lectura, 1);
    if(bytes_written == -1) {
        // Handle error
        RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Error en el envío del comando de lectura");
    }

    // unsigned char datos1[3][4], datos2[3][4];
    read_usb_float(USB1, datos1, 3);
    read_usb_float(USB2, datos2, 3);

    float corriente_1, corriente_2, corriente_3, corriente_4, corriente_5, corriente_6;
    if(comando_lectura == 'C'){
      std::memcpy(&corriente_1, datos1[0], sizeof(float));
      std::memcpy(&corriente_2, datos1[1], sizeof(float));
      std::memcpy(&corriente_3, datos1[2], sizeof(float));
      std::memcpy(&corriente_4, datos2[0], sizeof(float));
      std::memcpy(&corriente_5, datos2[1], sizeof(float));
      std::memcpy(&corriente_6, datos2[2], sizeof(float));
      // RCLCPP_INFO(rclcpp::get_logger("Listener"),
      // "Corriente Leida: %f/%f/%f/%f/%f/%f"
      //   ,corriente_1,corriente_2,corriente_3,corriente_4,corriente_5,corriente_6);

      actual_time = tiempo.seconds() - star_time;
      // RCLCPP_INFO(rclcpp::get_logger("Listener"),"Tiempo: %f",actual_time);

      std_msgs::msg::String mensaje;
      std::string separador = ",";
      mensaje.data = std::to_string(corriente_1) + separador +std::to_string(corriente_2) + separador + 
        std::to_string(corriente_3) + separador + std::to_string(corriente_4) + separador + 
        std::to_string(corriente_5) + separador + std::to_string(corriente_6);

      publisher->publish(mensaje); // Publicar la variable en el tópico

      rclcpp::spin_some(node);
    }
    

  ciclo_actual = ciclo_actual+1;
  if(ciclo_actual > ciclos_total){
    ciclo_actual = 0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SbotPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Comando para la primera placa
  // RCLCPP_INFO(rclcpp::get_logger("Listener"), "Corriente Leida: %f/%f/%f/%f/%f/%f",
  //   hw_commands_[0],hw_commands_[1],hw_commands_[2],
  //   hw_commands_[3],hw_commands_[4],hw_commands_[5]);

  float comando_f[] = {static_cast<float>(hw_commands_[0]*-180/3.14159), static_cast<float>(hw_commands_[1]*180/3.14159),static_cast<float>(hw_commands_[2]*180/3.14159)};
  unsigned char bytes_comando[3][4];
  std::memcpy(bytes_comando[0], &comando_f[0], sizeof(comando_f[0]));
  std::memcpy(bytes_comando[1], &comando_f[1], sizeof(comando_f[1]));
  std::memcpy(bytes_comando[2], &comando_f[2], sizeof(comando_f[2]));
  unsigned char cmd[] = "R";
  int n_written = 0, spot = 0;
  n_written = write_usb( USB1, cmd, 1 );
  for(int i = 0; i < 3; i++){
    spot = 3;
    do{
      n_written = write_usb( USB1, &bytes_comando[i][spot], 1 );
      spot -= n_written;
      // std::cout << "Spot luego: " << spot << "condición " << ((4- spot) > 4 && n_written > 0)<< std::endl;
    }while ((4- spot) < 5 && n_written > 0);
  }
  // Comando para la segunda placa
  comando_f[0] = static_cast<float>(hw_commands_[3]*-180/3.14159);
  comando_f[1] = static_cast<float>(hw_commands_[4]*180/3.14159);
  std::memcpy(bytes_comando[0], &comando_f[0], sizeof(comando_f[0]));
  std::memcpy(bytes_comando[1], &comando_f[1], sizeof(comando_f[1]));

  // Comandos del gripper son "Abrir" y "Cerrar" únicamente
  bytes_comando[2][3]=0;
  bytes_comando[2][2]=0;
  if(comando_viejo < hw_commands_[5]){ // Cerrar el gripper
    bytes_comando[2][1]=0;
    bytes_comando[2][0]=1;
    // RCLCPP_INFO(rclcpp::get_logger("Listener"),"Enviando Cierre");
  }else if(comando_viejo > hw_commands_[5]){ // Abrir el gripper
    bytes_comando[2][1]=1;
    bytes_comando[2][0]=0;
    // RCLCPP_INFO(rclcpp::get_logger("Listener"),"Enviando Apertura");
  }else{
    bytes_comando[2][1]=0;
    bytes_comando[2][0]=0;
    // RCLCPP_INFO(rclcpp::get_logger("Listener"),"Comando Gripper: %f",hw_commands_[5]);
  }
  
  n_written = write_usb( USB2, cmd, 1 );
  for(int i = 0; i < 3; i++){
    spot = 3;
    do{
      n_written = write_usb( USB2, &bytes_comando[i][spot], 1 );
      spot -= n_written;
    }while ((4- spot) < 5 && n_written > 0);
  }

  comando_viejo = hw_commands_[5];
  
  
  return hardware_interface::return_type::OK;
}

}  // namespace sbot_hi

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  sbot_hi::SbotPositionOnlyHardware, hardware_interface::SystemInterface)
