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

#include "ros2_control_example/rrbot.hpp"
#include "ros2_control_example/usb_functions.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


#include <cstdlib>


namespace ros2_control_example
{
hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  comando_viejo.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Activating ...please wait...");

  // Starting the conection via SERIAL
  /* Open File Descriptor */
  USB = open( "/dev/ttyACM0", O_RDWR| O_NOCTTY );
  if(conection_usb(USB) != 1){
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Finishing the conection via SERIAL

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  // unsigned char comando_lectura = 'I';
  // ssize_t bytes_written = write_usb(USB, &comando_lectura, 1);
  // if(bytes_written == -1) {
  //     // Handle error
  //     RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
  //     "Error en el envío del comando de lectura");
  // }

  // unsigned char respuesta;
  // do{
  //   read_usb(USB, &respuesta, 1);
  // } while(respuesta != 1);

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Deactivating ...please wait...");

  close(USB);

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*tiempo/*time*/, const rclcpp::Duration & /*periodo/*periodo*/)
{

  for (uint i = 3; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
  }

  if(ciclo_actual == ciclos_total){

    unsigned char comando_lectura = 'P';
    ssize_t bytes_written = write_usb(USB, &comando_lectura, 1);
    if(bytes_written == -1) {
        // Handle error
        RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Error en el envío del comando de lectura");
    }
    // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     "Pasó un ciclo de lectura");

    // unsigned char byte = 0;
    // int count= 0;
    // int dato = 0; 
    // unsigned char bytes[3][4];
    // unsigned char dato1[4],dato2[4],dato3[4];
    unsigned char datos[3][4];
    // int bandera = 0;


    read_usb_float(USB, datos, 3);
    // while (!bandera) {

    //   // Lectura del puerto
    //   read_usb_float(USB, &byte, 1);  // Leer un byte desde el puerto serie
    //   if(count < 4){
    //     dato1[count]=byte;
    //     // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     // "Dato n° 1: %d- count= %d",static_cast<int>(byte),count);
    //   }else if(count < 8){
    //     dato2[count-4]=byte;
    //     //  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     // "Dato n° 2: %d- count= %d",static_cast<int>(byte),count);
    //   }else if(count < 12){
    //     dato3[count-8]=byte;
    //     //  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     // "Dato n° 3: %d- count= %d",static_cast<int>(byte),count);
    //   }
    //   if(count > 10){
    //     //  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     // "Fuera- count= %d",count);
    //     bandera = 1;
    //   }
    //   count = count + 1;
    // }
    // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     "Bytes leidos: %s/%s/%s",bytes[0],bytes[1],bytes[2]);
    // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    // "Datos leidos: %f", periodo.seconds());

    float corriente_1, corriente_2, corriente_3;
    float intermedio;
    int HyT[3];

    if(comando_lectura == 'C'){
        std::memcpy(&corriente_1, datos[0], sizeof(float));
        std::memcpy(&corriente_2, datos[1], sizeof(float));
        std::memcpy(&corriente_3, datos[2], sizeof(float));
        // std::cout << "C: " << corriente_1 << '/' << corriente_2 << '/' << corriente_3 << std::endl;
    } else if(comando_lectura == 'P'){
        // Guardamos los ángulos (en grados) de cada junta en el estado (en radianes) correspondiente
        std::memcpy(&intermedio, datos[0], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[0] = intermedio*(3.1415/180); 
          // hw_states_[0] = static_cast<double>(-intermedio*(3.1415/180)); 
          hw_states_[3] = static_cast<double>(-intermedio*(3.1415/180)); 
        }
        // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        // "Intermedio 1: %d",intermedio);
        std::memcpy(&intermedio, datos[1], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[1] = intermedio*(3.1415/180); 
          // hw_states_[1] = static_cast<double>(intermedio*(3.1415/180)); 
          hw_states_[4] = static_cast<double>(intermedio*(3.1415/180)); 
        }
        std::memcpy(&intermedio, datos[2], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[2] = intermedio*(3.1415/180); 
          // hw_states_[2] = static_cast<double>(intermedio*(3.1415/180)); 
        }
        RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        // "Posición Leida: %g/%g/%g",hw_states_[0],hw_states_[1],hw_states_[2]);
        "Datos leidos: %g/%g",hw_states_[3],hw_states_[4]);
    } else{
        for(int i = 0; i < 3; i++){
          HyT[i] = static_cast<int>(datos[i][0]);
        }
        RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Datos leidos: %d/%d/%d",HyT[0],HyT[1],HyT[2]);
        // std::cout << "H/T: " << HyT[0] << '/' << HyT[1] << '/' << HyT[2] << std::endl;
    }
    
      
    // Limpiar la variable bytes
    // std::memset(bytes, 0, sizeof(bytes));


    // comando_lectura = 'C';
    // bytes_written = write_usb(USB, &comando_lectura, 1);
    // if(bytes_written == -1) {
    //     // Handle error
    //     RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     "Error en el envío del comando de lectura");
    // }

    // // bandera = 0;

    // read_usb_float(USB, datos, 3);
    // while (!bandera) {

    //   // Lectura del puerto
    //   read_usb_float(USB, &byte, 1);  // Leer un byte desde el puerto serie
    //   if(count < 4){
    //     dato1[count]=byte;
    //     // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     // "Dato n° 1: %d- count= %d",static_cast<int>(byte),count);
    //   }else if(count < 8){
    //     dato2[count-4]=byte;
    //     //  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     // "Dato n° 2: %d- count= %d",static_cast<int>(byte),count);
    //   }else if(count < 12){
    //     dato3[count-8]=byte;
    //     //  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     // "Dato n° 3: %d- count= %d",static_cast<int>(byte),count);
    //   }
    //   if(count > 10){
    //     //  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     // "Fuera- count= %d",count);
    //     bandera = 1;
    //   }
    //   count = count + 1;
    // }


    if(comando_lectura == 'C'){
        std::memcpy(&corriente_1, datos[0], sizeof(float));
        std::memcpy(&corriente_2, datos[1], sizeof(float));
        std::memcpy(&corriente_3, datos[2], sizeof(float));
        // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        // "Corriente Leida: %f/%f/%f",corriente_1,corriente_2,corriente_3);
    }
    // static int cuenta = 0;
    // static float corriente_anterior = 0;

    // if(corriente_anterior == corriente_1){
    //   RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //   "Se repitió un valor, cuenta %d", cuenta);
    //   cuenta = 0;
    // }else {
    //   cuenta +=1;
    // }
    // static float periodo_prom, periodo_suma = 0;
    // cuenta +=1;
    // periodo_suma += periodo.seconds();
    // periodo_prom = periodo_suma/cuenta;
    // if(cuenta >100){
    //   cuenta = 0;
    //   periodo_suma = 0;
    //   RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //   "Período promedio: %f,%f", periodo_prom, tiempo.seconds());

    // }
    // if(periodo.seconds() > 0.001){
    //   RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //   "Período: %f", periodo.seconds());
    // }

  //   corriente_anterior = corriente_1;
  }
  ciclo_actual = ciclo_actual+1;
  if(ciclo_actual > ciclos_total){
    ciclo_actual = 0;
  }
  // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
  //       "Ciclo actual: %d",ciclo_actual);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if(ciclo_actual == ciclos_total){
    //Write
    // unsigned char byte[info_.joints.size()];
    // unsigned char signo[info_.joints.size()];
    // int comando_nuevo_flag =0;
    
    // for (uint i = 0; i < hw_states_.size(); i++)
    // {
    //   if(hw_commands_[i] != comando_viejo[i]){
    //     comando_nuevo_flag = 1;
    //     comando_viejo[i] = hw_commands_[i];
    //     // Convertir el double en un char
    //     if(hw_commands_[i] < 0){ signo[i] = 1;}
    //     else{ signo[i] = 0;}
    //     // bytes_written = write_usb(USB, &signo, 1);
    //     byte[i] = static_cast<unsigned char>(abs(hw_commands_[i]*180/3.1415));
    //     // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     // "Comando nuevo%i:%d%d",i,signo[i],byte[i]);
    //   }
    // }

    // unsigned char comando[7] = {0};
    // if(comando_nuevo_flag){
    //   comando[0]='R';
    //   comando[1]=signo[0];
    //   comando[2]=byte[0];
    //   comando[3]=signo[1];
    //   comando[4]=byte[1];
    //   comando[5]=signo[2];
    //   comando[6]=byte[2];
    //   ssize_t bytes_written = write_usb(USB, comando, 7);
    //   if(bytes_written == -1) {
    //       // Handle error
    //       RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //       "Error en el envío de los comandos de las juntas");
    //   }else{
    //     RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    //     "Comando enviado: R%d%d/%d%d/%d%d",
    //     static_cast<int>(comando[1]),static_cast<int>(comando[2]),static_cast<int>(comando[3]),
    //     static_cast<int>(comando[4]),static_cast<int>(comando[5]),static_cast<int>(comando[6]));
    //   }
       
    // }

    // float comando_f[] = {static_cast<float>(hw_commands_[0]*180/3.14159), static_cast<float>(hw_commands_[1]*180/3.14159),static_cast<float>(hw_commands_[2]*180/3.14159)};
    float comando_f[] = {static_cast<float>(hw_commands_[3]*-180/3.14159), static_cast<float>(hw_commands_[4]*180/3.14159),static_cast<float>(hw_commands_[5]*180/3.14159)};
    unsigned char bytes_comando[3][4];
    std::memcpy(bytes_comando[0], &comando_f[0], sizeof(comando_f[0]));
    std::memcpy(bytes_comando[1], &comando_f[1], sizeof(comando_f[1]));
    std::memcpy(bytes_comando[2], &comando_f[2], sizeof(comando_f[2]));
    unsigned char cmd[] = "R";
    int n_written = 0, spot = 0;
    n_written = write_usb( USB, cmd, 1 );
    for(int i = 0; i < 3; i++){
      spot = 3;
      do{
        n_written = write_usb( USB, &bytes_comando[i][spot], 1 );
        std::cout << "Enviando comando " << i << ", byte: " << static_cast<int>(bytes_comando[i][spot]) << " spot: " << spot << std::endl;
        spot -= n_written;
        // std::cout << "Spot luego: " << spot << "condición " << ((4- spot) > 4 && n_written > 0)<< std::endl;
      }while ((4- spot) < 5 && n_written > 0);
    }
    
  }
  
  
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_example

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_example::RRBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)
