import os
from launch_ros.actions import Node
from launch import LaunchDescription 
# from ament_index_python.packages import get_package_share_path
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
def generate_launch_description():
    returnList = []
#     sbot_description_path = get_package_share_path('sbot_description')

#    # Bridge ROS topics and Gazebo messages for establishing communication
#     bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         parameters=[{
#             'config_file': os.path.join(sbot_description_path, 'config', 'bridge_config.yaml'),
#             'qos_overrides./tf_static.publisher.durability': 'transient_local',
#         }],
#         output='screen'
#     )

#       # create and return launch description object
#     return LaunchDescription(
#         [
#             bridge,
#         ]
#     )

    # Argumento de seleccion de modelo 
    control_selection = DeclareLaunchArgument(name='bridge', default_value='true', choices=['true', 'false'],
                                      description='True for enable, necesary for gazebo plugin control type')
    returnList.append(control_selection)
 # Gz - ROS Bridge
    bridgeDefaults = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition = IfCondition(LaunchConfiguration('bridge')),
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint States (IGN -> ROS2)
            '/world/default/model/scorbot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
             # Joint Commands (ROS2 -> GZ)
            '/model/scorbot/joint/base_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/scorbot/joint/elbow/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/scorbot/joint/shoulder/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/scorbot/joint/pitch/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/scorbot/joint/roll/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/scorbot/joint/finger_R_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/scorbot/joint/finger_L_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            ('/model/scorbot/joint/base_joint/cmd_pos', '/commands/base_joint'),
            ('/model/scorbot/joint/elbow/cmd_pos', '/commands/elbow'),
            ('/model/scorbot/joint/shoulder/cmd_pos', '/commands/shoulder'),
            ('/model/scorbot/joint/pitch/cmd_pos', '/commands/pitch'),
            ('/model/scorbot/joint/roll/cmd_pos', '/commands/roll'),
            ('/model/scorbot/joint/finger_R_joint/cmd_pos', '/commands/finger_R_joint'),
            ('/model/scorbot/joint/finger_L_joint/cmd_pos', '/commands/finger_L_joint'),
            ('/world/default/model/scorbot/joint_state', '/joint_states'), # "empty" para el modelo de mundo que estamos usando ya que se llama empty
        ],
        output='screen'
    )
    returnList.append(bridgeDefaults)


    # bridgeJointCommands = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     condition = IfCondition(LaunchConfiguration('bridge')),
    #     arguments=[
    #         # Joint Commands (ROS2 -> GZ)
    #         '/model/scorbot/joint/base_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
    #         '/model/scorbot/joint/elbow/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
    #         '/model/scorbot/joint/shoulder/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
    #         '/model/scorbot/joint/pitch/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
    #         '/model/scorbot/joint/roll/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
    #         '/model/scorbot/joint/finger_R_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
    #         '/model/scorbot/joint/finger_L_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
    #         # '/world/default/model/scorbot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
    #     ],
    #     remappings=[
    #         ('/model/scorbot/joint/base_joint/cmd_pos', '/commands/base_joint'),
    #         ('/model/scorbot/joint/elbow/cmd_pos', '/commands/elbow'),
    #         ('/model/scorbot/joint/shoulder/cmd_pos', '/commands/shoulder'),
    #         ('/model/scorbot/joint/pitch/cmd_pos', '/commands/pitch'),
    #         ('/model/scorbot/joint/roll/cmd_pos', '/commands/roll'),
    #         ('/model/scorbot/joint/finger_R_joint/cmd_pos', '/commands/finger_R_joint'),
    #         ('/model/scorbot/joint/finger_L_joint/cmd_pos', '/commands/finger_L_joint'),
    #         # ('/world/default/model/scorbot/joint_state', '/joint_states'),
    #     ],
    #     output='screen'
    # )
    # returnList.append(bridgeJointCommands)

    return LaunchDescription(returnList)