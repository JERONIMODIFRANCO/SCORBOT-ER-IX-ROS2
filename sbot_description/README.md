# SBOT_DESCRIPTION 
```
├── sbot_description
    ├── launch
    │      ├── jsp_gui.launch.py: joint state publisher gui for manual control of the joints
    │      ├── rsp.launch.py: robot state publisher necesary for obtain the actual state of the robot
    │      ├── rviz_gui.launch.xml: visualization and joint state publisher gui manipulation
    │      └── rviz.launch.py: visualization of robot
    │
    ├── meshes
    │      └──  ... all the meshes that compose the robot
    │ 
    ├── rviz
    │      ├── config.rviz: base configuration for rviz 
    │      └── moveit.rviz: moveit configuration for rviz
    │
    ├── urdf
    │      ├──caja.xacro: not in use
    │      ├──macros_links_y_joints.xacro: macros to simplify the description of the robot
    │      ├──physical_properties:  some physical properties required for proper operation in Gazebo
    │      ├──sbot_arm.urdf.xacro: arm definition
    │      ├──sbot_gripper.urdf.xacro: gripper definition
    │      └──scorbot.urdf.xacro: base robot definition (includes sbot_gripper and sbot_arm)
    │
    ├── README.md
    ├── CMakeLists.txt
    └── package.xml
```
