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
  comando_viejo.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
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

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SbotPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("SbotPositionOnlyHardware"), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("SbotPositionOnlyHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

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
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SbotPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("SbotPositionOnlyHardware"), "Activating ...please wait...");

  // Starting the conection via SERIAL
  /* Open File Descriptor */
  USB1 = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );
  //  RCLCPP_INFO(
  //   rclcpp::get_logger("SbotPositionOnlyHardware"), "Activado 1 / %d",USB1);
  USB2 = open( "/dev/ttyACM0", O_RDWR| O_NOCTTY );
  // USB2=1;
  //  RCLCPP_INFO(
  //   rclcpp::get_logger("SbotPositionOnlyHardware"), "Activado 2 / %d",USB2);

  //if(conection_usb(USB1,USB2) != 1){
  // RCLCPP_INFO(
  //  rclcpp::get_logger("SbotPositionOnlyHardware"), "Fallo");
  //  return hardware_interface::CallbackReturn::ERROR;
  //}
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

  // unsigned char respuesta;
  // do{
  //   read_usb(USB, &respuesta, 1);
  // } while(respuesta != 1);

  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SbotPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("SbotPositionOnlyHardware"), "Deactivating ...please wait...");

  close(USB1);
  //close(USB2);

  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SbotPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*periodo*/)
{

  // for (uint i = 5; i < hw_states_.size(); i++)
  // {
  //   // Simulate RRBot's movement
  //   hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
  // }
  
  // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
  //       "Estoy leyendo");

  if(ciclo_actual == ciclos_total){
	//  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
  //       "Ciclo");
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
    // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     "Pasó un ciclo de lectura");

    // unsigned char byte = 0;
    // int count= 0;
    // int dato = 0; 
    // unsigned char bytes[3][4];
    // unsigned char dato1[4],dato2[4],dato3[4];
    unsigned char datos1[3][4], datos2[3][4];
    // int bandera = 0;
    
    // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     "Leo");


    read_usb_float(USB1, datos1, 3);
    read_usb_float(USB2, datos2, 3);
    // while (!bandera) {

    //   // Lectura del puerto
    //   read_usb_float(USB, &byte, 1);  // Leer un byte desde el puerto serie
    //   if(count < 4){
    //     dato1[count]=byte;
    //     // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     // "Dato n° 1: %d- count= %d",static_cast<int>(byte),count);
    //   }else if(count < 8){
    //     dato2[count-4]=byte;
    //     //  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     // "Dato n° 2: %d- count= %d",static_cast<int>(byte),count);
    //   }else if(count < 12){
    //     dato3[count-8]=byte;
    //     //  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     // "Dato n° 3: %d- count= %d",static_cast<int>(byte),count);
    //   }
    //   if(count > 10){
    //     //  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     // "Fuera- count= %d",count);
    //     bandera = 1;
    //   }
    //   count = count + 1;
    // }
    // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     "Bytes leidos: %s/%s/%s",bytes[0],bytes[1],bytes[2]);
    // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    // "Datos leidos: %f", periodo.seconds());

    float corriente_1, corriente_2, corriente_3;
    float intermedio;
    int HyT[3];
    // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     "Paso");

    if(comando_lectura == 'C'){
        std::memcpy(&corriente_1, datos1[0], sizeof(float));
        std::memcpy(&corriente_2, datos1[1], sizeof(float));
        std::memcpy(&corriente_3, datos1[2], sizeof(float));
        // std::cout << "C: " << corriente_1 << '/' << corriente_2 << '/' << corriente_3 << std::endl;
    } else if(comando_lectura == 'P'){
        // Guardamos los ángulos (en grados) de cada junta en el estado (en radianes) correspondiente
        std::memcpy(&intermedio, datos1[0], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[0] = intermedio*(3.1415/180); 
          hw_states_[0] = static_cast<double>(-intermedio*(3.1415/180)); 
          // hw_states_[3] = static_cast<double>(-intermedio*(3.1415/180)); 
        }
        // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        // "Intermedio 1: %d",intermedio);
        std::memcpy(&intermedio, datos1[1], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[1] = intermedio*(3.1415/180); 
          hw_states_[1] = static_cast<double>(intermedio*(3.1415/180)); 
          // hw_states_[4] = static_cast<double>(intermedio*(3.1415/180)); 
        }
        std::memcpy(&intermedio, datos1[2], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[2] = intermedio*(3.1415/180); 
          hw_states_[2] = static_cast<double>(intermedio*(3.1415/180)); 
        }
        // Guardamos los ángulos (en grados) de cada junta en el estado (en radianes) correspondiente
        std::memcpy(&intermedio, datos2[0], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[0] = intermedio*(3.1415/180); 
          // hw_states_[0] = static_cast<double>(-intermedio*(3.1415/180)); 
          hw_states_[3] = static_cast<double>(-intermedio*(3.1415/180)); 
        }
        // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        // "Intermedio 1: %d",intermedio);
        std::memcpy(&intermedio, datos2[1], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          // hw_states_[1] = intermedio*(3.1415/180); 
          // hw_states_[1] = static_cast<double>(intermedio*(3.1415/180)); 
          hw_states_[4] = static_cast<double>(intermedio*(3.1415/180)); 
        }
        std::memcpy(&intermedio, datos2[2], sizeof(float));
        if((intermedio*(3.1415/180) < 6.28) && (intermedio*(3.1415/180) > -6.28)){
          hw_states_[5] = hw_states_[5] + (hw_commands_[5] - hw_states_[5])/100;
          // hw_states_[2] = static_cast<double>(intermedio*(3.1415/180)); 
        }
        RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Posición Leida: %g/%g/%g/%g/%g/%g",hw_states_[0],hw_states_[1],hw_states_[2],hw_states_[3],hw_states_[4],hw_states_[5]);
    } else{
        for(int i = 0; i < 3; i++){
          HyT[i] = static_cast<int>(datos1[i][0]);
        }
        RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        "Datos leidos: %d/%d/%d",HyT[0],HyT[1],HyT[2]);
        // std::cout << "H/T: " << HyT[0] << '/' << HyT[1] << '/' << HyT[2] << std::endl;
    }
    
      
    // Limpiar la variable bytes
    // std::memset(bytes, 0, sizeof(bytes));


    // comando_lectura = 'C';
    // bytes_written = write_usb(USB, &comando_lectura, 1);
    // if(bytes_written == -1) {
    //     // Handle error
    //     RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     "Error en el envío del comando de lectura");
    // }

    // // bandera = 0;

    // read_usb_float(USB, datos, 3);
    // while (!bandera) {

    //   // Lectura del puerto
    //   read_usb_float(USB, &byte, 1);  // Leer un byte desde el puerto serie
    //   if(count < 4){
    //     dato1[count]=byte;
    //     // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     // "Dato n° 1: %d- count= %d",static_cast<int>(byte),count);
    //   }else if(count < 8){
    //     dato2[count-4]=byte;
    //     //  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     // "Dato n° 2: %d- count= %d",static_cast<int>(byte),count);
    //   }else if(count < 12){
    //     dato3[count-8]=byte;
    //     //  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     // "Dato n° 3: %d- count= %d",static_cast<int>(byte),count);
    //   }
    //   if(count > 10){
    //     //  RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     // "Fuera- count= %d",count);
    //     bandera = 1;
    //   }
    //   count = count + 1;
    // }


    if(comando_lectura == 'C'){
        std::memcpy(&corriente_1, datos1[0], sizeof(float));
        std::memcpy(&corriente_2, datos1[1], sizeof(float));
        std::memcpy(&corriente_3, datos1[2], sizeof(float));
        // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
        // "Corriente Leida: %f/%f/%f",corriente_1,corriente_2,corriente_3);
    }
    // static int cuenta = 0;
    // static float corriente_anterior = 0;

    // if(corriente_anterior == corriente_1){
    //   RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
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
    //   RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //   "Período promedio: %f,%f", periodo_prom, tiempo.seconds());

    // }
    // if(periodo.seconds() > 0.001){
    //   RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //   "Período: %f", periodo.seconds());
    // }

  //   corriente_anterior = corriente_1;
  // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
  //       "Fin");
  }
  ciclo_actual = ciclo_actual+1;
  if(ciclo_actual > ciclos_total){
    ciclo_actual = 0;
  }
  // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
  //       "Ciclo actual: %d",ciclo_actual);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SbotPositionOnlyHardware::write(
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
    //     // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
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
    //       RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //       "Error en el envío de los comandos de las juntas");
    //   }else{
    //     RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     "Comando enviado: R%d%d/%d%d/%d%d",
    //     static_cast<int>(comando[1]),static_cast<int>(comando[2]),static_cast<int>(comando[3]),
    //     static_cast<int>(comando[4]),static_cast<int>(comando[5]),static_cast<int>(comando[6]));
    //   }
       
    // }
    // RCLCPP_INFO(rclcpp::get_logger("SbotPositionOnlyHardware"),
    //     "Leer");

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
  sbot_hi::SbotPositionOnlyHardware, hardware_interface::SystemInterface)
