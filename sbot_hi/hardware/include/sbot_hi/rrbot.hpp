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

#ifndef SBOT_HI__RRBOT_HPP_
#define SBOT_HI__RRBOT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sbot_hi/visibility_control.h"


namespace sbot_hi
{
class ScorbotPositionOnlyHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ScorbotPositionOnlyHardwareInterface);

  SBOT_HI_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  SBOT_HI_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  // Devuelve el estado de las juntas hacia el controller_manager
  SBOT_HI_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Devuelve los comandos de las juntas desde el controller_manager
  SBOT_HI_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Abrimos los USBs para la comunicación, inicializamos los comandos y ejecutamos la init del brazo
  SBOT_HI_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Cerramos los USBs
  SBOT_HI_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Lectura de los estados de las juntas, corrientes, y banderas de homeswitches y temperaturas
  SBOT_HI_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SBOT_HI_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // Variables para la comunicaciones
  const char * puerto_placa_1 = "/dev/ttyS2";
  const char * puerto_placa_2 = "/dev/ttyACM0";
  int USB1, USB2; //identificador del usb utilizado
  std::vector<double> comando_viejo;
  int ciclo_actual = 0;
  int ciclos_total = 1;
};

}  // namespace sbot_hi

#endif  // SBOT_HI__RRBOT_HPP_
