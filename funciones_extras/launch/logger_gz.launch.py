import os
from launch_ros.actions import Node
from launch import LaunchDescription 
# from ament_index_python.packages import get_package_share_path
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
def generate_launch_description():
    returnList = []

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
        ],
        remappings=[
            ('/clock', '/clock_gz'),
        ],
        output='screen'
    )
    returnList.append(bridgeDefaults)

    logger_gz = Node(
        package='funciones_extras',
        executable='logger_gz',
        output='screen'
    )
    returnList.append(logger_gz)



    return LaunchDescription(returnList)