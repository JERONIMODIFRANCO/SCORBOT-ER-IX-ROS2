# SCORBOT-ER-IX-ROS2

*Ros2 packages for a control uptdate of a SCORBOT-ER-IX robot arm*

 *Packages developed for ROS2 Humble, Ignition Fortress and Linux Ubuntu 22.04LTS*


# PREREQUISITOS E INSTALACIONES NECESARIAS 

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


**CLONE AND SOURCE REPOSITORY**

* Crear Workspace y Clonar repositorio:
~~~
    mkdir -p ~/sbot/src
    cd ~/sbot
    git clone -b refactor_v1 git@github.com:LAC-FCEIA-UNR/Scorbot-ERIX-ROS.git src #chequear
~~~
* Compilar y Source
~~~
    cd ~/sbot
    colcon build
    source ~/sbot/install/setup.sh
~~~
### IMPORTANTE "MATAR" (Ctrl+C) TODOS LOS PROCESOS ANTES DE LANZAR OTRO EJECUTABLE PARA EVITAR INCONSISTENCIAS O NODOS EJECUTANDOSÉ EN EL BACKGROUND

**ESTRUCTURA**

SCORBOT-ER-IX-ROS2
```
├── sbot_description
│   ├── launch
│   ├── meshes
│   ├── rviz
│   ├── urdf
│   ├── README.md
│   ├── CMakeLists.txt
│   └── package.xml
│
├── sbot_gazebo
│   ├── config
│   ├── launch
│   ├── models
│   ├── worlds
│   ├── README.md
│   ├── CMakeLists.txt
│   └── package.xml
│
├── sbot_moveit
│   ├── config
│   ├── launch
│   ├── .setup_assistant
│   ├── README.md
│   ├── CMakeLists.txt
│   └── package.xml
│
├── sbot_moveit_gazebo
│   ├── config
│   ├── launch
│   ├── .setup_assistant
│   ├── README.md
│   ├── CMakeLists.txt
│   └── package.xml
│
├── README.md
├── LICENSE
└── .gitignore
```

 * **[sbot_description](sbot_description/readme.md)**: Contiene la descripción para **ROS2** del modelo en formato URDF, incluidas las meshses (exportadas directamente desde SolidWorks) y ejecutables de `rviz` para visaulización y debugeo inicial del robot.

 * **[sbot_gazebo](sbot_gazebo/readme.md)**: Contiene los archivos necesarios para la simulación basíca de control en Gazebo/Ignition. Los modelos URDF se importan de la descripción contenida en *sbot_description* y se agregan los plugins de GAZEBO (controladores PID de las juntas con las ganancias correspondientes). Además existen ejecutables para realizar el bridge, necesario para comunicar Ignition con ROS2. Existe un launcher para iniciar gazebo, y otro para realizar el spawn del brazo dentro de gazeob y demás nodos necesarios para la correcta simulacion en Ignition, así como tambien la visualización paralela en RVIZ2.
   
 * **[sbot_moveit](sbot_moveit/readme.md)**: Contiene los archivos generados en una estructura predefinida por el setup assistant de MOVEIT2. Existe un conjunto de launchs que cumplen diferentes funciones, y posee una estructura modular generada que permite el agregado o modificacion reducida de lineas de codigo para, en conjunto con ROS2CONTROL permitir el uso de controladores existentes o desarrollados para realizar controles de alto nivel. Mediante RVIZ se permite controlar las distintas juntas que componen al robot para posteriormente ser controladas, valga la redundancia, por el control definido. 

 * **[sbot_moveit_gazebo](sbot_moveit_gazebo/readme.md)**: Este paquete es similar al anterior con el añadido configuraciones para la ejecución de la simulación del control de alto nivel sobre Gazebo. Para ello se modifica la descripción del robot, y se añaden ejecutables para inciar gazebo y spawnear el robot.
