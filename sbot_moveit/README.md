# SBOT_MOVEIT
```
├── sbot_moveit
    │
    ├── config
    │      ├── initial_positions.yaml: Configuration for initial robot positions.
    │      ├── joint_limits.yaml: Joint limits configuration for the robot.
    │      ├── kinematics.yaml: Kinematics configuration for the robot.
    │      ├── moveit_controllers: Controller configurations for MoveIt.
    │      ├── moveit.rviz: MoveIt visualization configuration in RViz.
    │      ├── pilz_cartesian_limits.yaml: Specific Cartesian limits for the Scorbot robot.
    │      ├── ros2_controllers.yaml: Controller configuration for ROS 2.
    │      ├── scorbot.ros2_control.xacro: ROS2 control configuration in xacro format.
    │      ├── scorbot.sdf: control groups and other definitions.
    │      └── scorbot.urdf.xacro: model that includes the base description and the scorbot.ros2_control.xacro for properly operation.
    │     
    ├── launch
    │      ├── demo.launch: Launch file to dynamically control the robot in RViz.
    │      ├── move_group.launch.py: Launch file for proper management of ROS 2 controllers.
    │      ├── moveit_rviz.launch.py: RViz launch file.
    │      ├── rsp.launch.py: robot state publisher.
    │      ├── setup_assistant.launch.py: Launch file to reconfigure the file.
    │      ├── spawn_controllers.launch.py: Launch file to spawn controllers.
    │      ├── static_virtual_joint_tfs.launch.py: Launch file for virtual joint transformations.
    │      └── warehouse_db.launch.py: Launch file for the warehouse database.
    │
    ├── .setup_assistant
    ├── README.md
    ├── CMakeLists.txt
    └── package.xml
```
## Launch file principal 
```bash
ros2 launch sbot_moveit demo.launch.py
```
