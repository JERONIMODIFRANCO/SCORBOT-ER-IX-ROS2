# SBOT_GAZEBO
```
├── sbot_gazebo
    │
    ├── config
    │      ├── gz_plugin_control.xacro: ignition plugin configuration for joint position control
    │      └── gz_ros2_control.xacro: ros2 control config for control in ignition (not in use, future implementation or deletion)
    │     
    ├── launch
    │      ├── bridge.launch.py: bridge between ROS2 and Ignition
    │      ├── controllers.launch.py: launch file to proper manage of the ros2 controllers (not in use)
    │      ├── spawn_robot_gazebo.launch.xml: implementation of ignition plugin contol for scorbot + rviz 
    │      ├── spawn_robot.launch.py: launch file to spawn the robot in ignition
    │      └── start_world.launch.py: launch file to initiate ignition world
    │
    ├── models
    │      └──... definition of ignition models (not in use)
    │   
    ├── worlds
    │      └──... definition of ignition worlds
    │
    ├── README.md
    ├── CMakeLists.txt
    └── package.xml
```
## Launch file principal 
```bash
ros2 launch sbot_gazebo spawn_robot_gazebo.launch.xml
```
Para ver los argumentos modificables y setear los mismos
```bash
ros2 launch sbot_gazebo spawn_robot_gazebo.launch.xml --show-args
ros2 launch sbot_gazebo spawn_robot_gazebo.launch.xml arg:=value
```
Para controlar en gazebo las juntas se debe publicar sobre el topico correspondiente
```
ros2 topic pub /commands/<joint_name> std_msgs/msg/Float64 'data: "<valor>"'
```
Los posibles nombres de las juntas <joint_name\> a controlar y el tipo (en paréntesis) son:

1. *base_joint* (revolución): Junta de la base
   ```bash
   ros2 topic pub /commands/base_joint std_msgs/msg/Float64 "data: '<valor>'"
2. *shoulder* (revolución): Articulación base-brazo
   ```bash
   ros2 topic pub /commands/shoulder std_msgs/msg/Float64 "data: '<valor>'"
3. *elbow* (revolución): Articulación brazo-antebrazo
   ```bash
   ros2 topic pub /commands/elbow std_msgs/msg/Float64 "data: '<valor>'"
4. *pitch* (revolución): Articulación muñeca
   ```bash
   ros2 topic pub /commands/pitch std_msgs/msg/Float64 "data: '<valor>'"
5. *roll* (revolución): Rotación del End Effector
   ```bash
   ros2 topic pub /commands/roll std_msgs/msg/Float64 "data: '<valor>'"
6. *finger_L_joint* (prismática): Dedos del gripper (ambos juntos)
   ```bash
   ros2 topic pub /commands/finger_L_joint std_msgs/msg/Float64 "data: '<valor>'"


El <valor\> pasado está en radianes para las juntas de revolución y en metros para las prismáticas.


* Comentario sobre el movimiento de los dedos:
  
Las 2 juntas están limitadas en su rango de movimiento. Los límites de movimiento están limitados a 0 (dedos abiertos) como mínimo y 0.024 (en metros, dedos cerrados) como máximo.
