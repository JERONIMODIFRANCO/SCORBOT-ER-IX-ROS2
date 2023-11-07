# SCORBOT-ER-IX-ROS2
Ros2 packages for a control uptdate of a SCORBOT-ER-IX robot arm
Packages developed for ROS2 Humble, Ignition Fortress and Linux 

# PAQUETES, INSTALACIONES y COMENTARIOS 
ROS2 HUMBLE
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
Al instalar ros 2 (no instalar ros base), si el ros desktop y ros tools

// Para ejecutar ros en una terminal hay q bashearlo, alternativamente se puede añadir al bashrc para que cada vez que se abra la ventana de comandos se genere el source, de esta forma ros puede ver los paquetes
'source /opt/ros/humble/setup.bash'

MOVEIT
https://moveit.ros.org/install-moveit2/binary/

ROS2 CONTROL
 'sudo apt install ros-humble-ros2-control'
 sudo apt install ros-humble-ros2-controllers

Hasta aca iba con sbot_description y necesite :
 sudo apt install ros-humble-joint-state-publisher
 sudo apt install ros-humble-joint-state-publisher-gui

PARA MOVEIT Y RESTO 
 sudo apt install ros-humble-gripper-controllers


GAZEBO IGNITION FORTRESS
instalando gazebo fortress desde:
https://gazebosim.org/docs/fortress/install_ubuntu
 sudo apt install ros-humble-ros-ign
 sudo apt install ros-humble-gazebo-ros-pkgs 

ESTO NO SE DE DONDE SALIO
. ~/workspace/install/setup.bash
https://gazebosim.org/docs/fortress/install_ubuntu_src

 
Por el momento no los instale

 ros-humble-ign-ros2-control
 ros-humble-gazebo-ros2-control
  
 
 ESTOS DOS SUPUESTAMENTE VIENEN CON OTROS ARCHIVOS, EL SEGUNDO NO ESTOY SEGURO EL PRIMERO SI
 joint-state-publisher
 ros-humble-rqt-joint-trajectory-controller
 source ~/scorbot_ws/install/setup.sh

BASHRC 2/11
export GZ_VERSION=fortress
export ROS_PACKAGE_PATH=/opt/ros/humble/share:$ROS_PACKAGE_PATH #SOURCE PARA QUE GAZEBO LEA LOS PAQUETES QUE TRAE POR DEFECTO

export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/jdf/scorbot_ws/install/sbot_description/share/sbot_description/meshes
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/jdf/sbot/install/sbot_description/share/sbot_description/meshes
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib

source /opt/ros/humble/setup.bash #SOURCE DE ROS2 HUMBLE
source ~/sbot/install/setup.bash #SOURCE DE ENTORNO DE TRABAJO DEL PROYECTO

source ~/ws_moveit2/install/setup.bash 



COMANDOS UTILES:
colcon build --packages-select sbot_moveit para construir un paquete solo
source /direcciondelpaquete/install/setup.bash para hacer visible un entorno para ros
