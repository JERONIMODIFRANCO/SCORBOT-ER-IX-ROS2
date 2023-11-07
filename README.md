# SCORBOT-ER-IX-ROS2

*Ros2 packages for a control uptdate of a SCORBOT-ER-IX robot arm*

 *Packages developed for ROS2 Humble, Ignition Fortress and Linux Ubuntu 22.04LTS*


# PAQUETES, INSTALACIONES y COMENTARIOS 

## ROS2 HUMBLE

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Al instalar ros 2 (no instalar ros base), si el ros desktop y ros tools


## MOVEIT

https://moveit.ros.org/install-moveit2/binary/

## ROS2 CONTROL

 ``` sudo apt install ros-humble-ros2-control``` 
 
```  sudo apt install ros-humble-ros2-controllers```  

## GAZEBO IGNITION FORTRESS

https://gazebosim.org/docs/fortress/install_ubuntu


## ADICIONALES 

 ``` sudo apt install ros-humble-gripper-controllers``` 
 
 ``` sudo apt install ros-humble-joint-state-publisher``` 
 
 ``` sudo apt install ros-humble-joint-state-publisher-gui``` 
 
 ``` sudo apt install ros-humble-ros-ign``` 
 
 ``` sudo apt install ros-humble-gazebo-ros-pkgs``` 
 
 ``` ros-humble-ign-ros2-control #solo deberia ser este y no el que sigue``` 
 
 ``` ros-humble-gazebo-ros2-control``` 
 

## ENVIROMENT VARIABLES  (recomendable colocarlas en el bashrc)

An environment variable is a dynamic-named value that can affect the way running processes will behave on a computer.

Environment variables are set when you open a new Shell session

The **export** command, on the other hand, provides the ability to update the current Shell session about the change you made to the exported variable

``` export GZ_VERSION=fortress #setear fortress como version de gazebo``` 

``` export ROS_PACKAGE_PATH=/opt/ros/humble/share:$ROS_PACKAGE_PATH #para que ros acceda a los paquetes que trae por defecto``` 

``` export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/sbot/install/sbot_description/share/sbot_description/meshes #para que gazebo pueda acceder a las meshes de descripcion del scorbot``` 

``` export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib #para que ignition acceda a los plugins```  

``` source /opt/ros/humble/setup.bash #SOURCE DE ROS2 HUMBLE``` 

``` source ~/sbot/install/setup.bash #SOURCE DE ENTORNO DE TRABAJO DEL PROYECTO``` 


ESTOS NO , FALTA CONFIRMAR 

``` export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/sbot/install/sbot_description/share/sbot_description/meshes``` 

``` export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib``` 

``` export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib``` 



# COMANDOS UTILES:


