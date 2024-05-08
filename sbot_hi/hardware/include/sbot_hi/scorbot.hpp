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

#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp" 


namespace sbot_hi
{
class SbotPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SbotPositionOnlyHardware);

  SBOT_HI_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  SBOT_HI_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SBOT_HI_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SBOT_HI_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SBOT_HI_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SBOT_HI_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

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
  std::vector<double> hw_states_velocities_;

  // Variables para la comunicaciones
  int USB1, USB2; //identificador del usb utilizado
  double comando_viejo;
  int ciclo_actual = 0;
  int ciclos_total = 1;

  //Varialbes para el nodo publicador
  std::shared_ptr<rclcpp::Node> node;
  decltype(node->create_publisher<std_msgs::msg::String>("corriente", 10)) publisher; // Declaración de la variable publisher
};

}  // namespace sbot_hi

#endif  // SBOT_HI__RRBOT_HPP_
