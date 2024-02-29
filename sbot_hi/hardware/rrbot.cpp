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

#include "sbot_hi/rrbot.hpp"
#include "sbot_hi/usb_functions.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


#include <cstdlib>


namespace sbot_hi
{
hardware_interface::CallbackReturn ScorbotPositionOnlyHardwareInterface::on_init(
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
  comando_viejo.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // SbotPositionOnly has exactly one state and command interface on each joint
    // Veo si cada junta posee una única interfaz de comando
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Veo si dicha interfaz es de posición
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Veo si cada junta posee una única interfaz de estado
    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Veo si dicha interfaz es de posición
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ScorbotPositionOnlyHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"), "Configuring ...please wait...");

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Devuelve el estado de las juntas hacia el controller_manager
std::vector<hardware_interface::StateInterface>
ScorbotPositionOnlyHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

// Devuelve los comandos de las juntas desde el controller_manager
std::vector<hardware_interface::CommandInterface>
ScorbotPositionOnlyHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

// Abrimos los USBs para la comunicación, inicializamos los comandos y ejecutamos la init del brazo
hardware_interface::CallbackReturn ScorbotPositionOnlyHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"), "Activating ...please wait...");

  // Starting the conection via SERIAL
  /* Open File Descriptor */
  USB2 = open( puerto_placa_1, O_RDWR| O_NOCTTY );
  USB1 = open( puerto_placa_2, O_RDWR| O_NOCTTY );
  if(conection_usb(USB1,USB2) != 1){
    return hardware_interface::CallbackReturn::ERROR;
  }

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  // Enviamos una 'I' para que el brazo empiece a inicializar
  unsigned char comando_lectura = 'I';
  ssize_t bytes_written = write_usb(USB1, &comando_lectura, 1);
  if(bytes_written == -1) {
      // Handle error
      RCLCPP_INFO(rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"),
      "Error en el envío del comando de inicialización junta 1");
  }
  bytes_written = write_usb(USB2, &comando_lectura, 1);
  if(bytes_written == -1) {
      // Handle error
      RCLCPP_INFO(rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"),
      "Error en el envío del comando de inicialización junta 2");
  }

  RCLCPP_INFO(rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Cerramos los USBs
hardware_interface::CallbackReturn ScorbotPositionOnlyHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"), "Deactivating ...please wait...");

  close(USB1);
  close(USB2);

  RCLCPP_INFO(rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"), "Successfully deactivated!");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Lectura de los estados de las juntas, corrientes, y banderas de homeswitches y temperaturas
hardware_interface::return_type ScorbotPositionOnlyHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*periodo*/)
{
  // Ejecuto en un ciclo de scan la lectura, y en el otro la escritura
  if(ciclo_actual == ciclos_total){

    // Envio el comando 'P' para que me devuelvan el estado de las juntas
    unsigned char comando_lectura = 'P';
    ssize_t bytes_written = write_usb(USB1, &comando_lectura, 1);
    if(bytes_written == -1) {
        // Handle error
        RCLCPP_INFO(rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"),
        "Error en el envío del comando de lectura");
    }
    bytes_written = write_usb(USB2, &comando_lectura, 1);
    if(bytes_written == -1) {
        // Handle error
        RCLCPP_INFO(rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"),
        "Error en el envío del comando de lectura");
    }
    
    // Leo 3 flotantes desde un puerto y 3 desde el otro
    unsigned char datos1[3][4], datos2[3][4];
    read_usb_float(USB1, datos1, 3); // Estados juntas 1, 2 y 3
    read_usb_float(USB2, datos2, 3); // Estados juntas 4, 5 y gripper
    

    
    float intermedio;
    
    if(comando_lectura == 'P'){
      // Guardamos los ángulos (en grados) de cada junta en el estado (en radianes) correspondiente
      std::memcpy(&intermedio, datos1[0], sizeof(float));
      // Verifico si el dato obtenido tiene sentido dentro del sistema
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
        // hw_states_[5] = static_cast<double>(intermedio*(3.1415/180)); 
      }
      RCLCPP_INFO(rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"),
      "Posición Leida: %g/%g/%g/%g/%g/%g",hw_states_[0],hw_states_[1],hw_states_[2],hw_states_[3],hw_states_[4],hw_states_[5]);
    }
    
    
    // Lectura y almacenaje de la corriente
    // comando_lectura = 'C';
    // bytes_written = write_usb(USB, &comando_lectura, 1);
    // if(bytes_written == -1) {
    //     // Handle error
    //     RCLCPP_INFO(rclcpp::get_logger("ScorbotPositionOnlyHardwareInterface"),
    //     "Error en el envío del comando de lectura");
    // }

    // if(comando_lectura == 'C'){
    //     std::memcpy(&corriente_1, datos1[0], sizeof(float));
    //     std::memcpy(&corriente_2, datos1[1], sizeof(float));
    //     std::memcpy(&corriente_3, datos1[2], sizeof(float));
    // }
    
  }
  ciclo_actual = ciclo_actual+1;
  if(ciclo_actual > ciclos_total){
    ciclo_actual = 0;
  }
  
  return hardware_interface::return_type::OK;
}

// Envío de comandos 
hardware_interface::return_type ScorbotPositionOnlyHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Ver si esto se puede sacar o no
  if(ciclo_actual == ciclos_total){
    
    // Almaceno los comandos de las juntas 1, 2 y 3
    float comando_f[] = {static_cast<float>(hw_commands_[0]*-180/3.14159), static_cast<float>(hw_commands_[1]*180/3.14159),static_cast<float>(hw_commands_[2]*180/3.14159)};
    // Los paso de flotante a 4 bytes
    unsigned char bytes_comando[3][4];
    std::memcpy(bytes_comando[0], &comando_f[0], sizeof(comando_f[0]));
    std::memcpy(bytes_comando[1], &comando_f[1], sizeof(comando_f[1]));
    std::memcpy(bytes_comando[2], &comando_f[2], sizeof(comando_f[2]));
    // Envío una R para avisar que voy a enviar los comandos para las juntas
    unsigned char cmd[] = "R";
    int n_written = 0, spot = 0;
    n_written = write_usb( USB1, cmd, 1 );

    // Envio (desde el byte más significativo al menos) los comandos de las 3 primeras juntas
    for(int i = 0; i < 3; i++){
      spot = 3;
      do{
        n_written = write_usb( USB1, &bytes_comando[i][spot], 1 );
        std::cout << "Enviando comando " << i << ", byte: " << static_cast<int>(bytes_comando[i][spot]) << " spot: " << spot << std::endl;
        spot -= n_written;
        // std::cout << "Spot luego: " << spot << "condición " << ((4- spot) > 4 && n_written > 0)<< std::endl;
      }while ((4- spot) < 5 && n_written > 0);
    }
    comando_f[0] = static_cast<float>(hw_commands_[3]*-180/3.14159);
    comando_f[1] = static_cast<float>(hw_commands_[4]*180/3.14159);
    comando_f[2] = static_cast<float>(hw_commands_[5]*180/3.14159); 
    std::memcpy(bytes_comando[0], &comando_f[0], sizeof(comando_f[0]));
    std::memcpy(bytes_comando[1], &comando_f[1], sizeof(comando_f[1]));
    std::memcpy(bytes_comando[2], &comando_f[2], sizeof(comando_f[2]));
    n_written = write_usb( USB2, cmd, 1 );
    for(int i = 0; i < 3; i++){
      spot = 3;
      do{
        n_written = write_usb( USB2, &bytes_comando[i][spot], 1 );
        std::cout << "Enviando comando " << i << ", byte: " << static_cast<int>(bytes_comando[i][spot]) << " spot: " << spot << std::endl;
        spot -= n_written;
        // std::cout << "Spot luego: " << spot << "condición " << ((4- spot) > 4 && n_written > 0)<< std::endl;
      }while ((4- spot) < 5 && n_written > 0);
    }
  }
  
  
  return hardware_interface::return_type::OK;
}

}  // namespace sbot_hi

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  sbot_hi::ScorbotPositionOnlyHardwareInterface, hardware_interface::SystemInterface)
