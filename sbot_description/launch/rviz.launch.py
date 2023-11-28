from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    default_package_path = get_package_share_path('sbot_description')
    default_rviz_config_path = default_package_path / 'rviz/config.rviz'
    moveit_rviz_config_path = default_package_path / 'rviz/moveit.rviz'

    moveit_rviz_arg = DeclareLaunchArgument(name='moveit_rviz', default_value='false', 
                                    choices=['true', 'false'],
                                    description='Flag to enable RVIZ with Moveit Plugins')
    
    # rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
    #                                  description='Absolute path to rviz config file',
    #                                  choices=[str(default_rviz_config_path),str(moveit_rviz_config_path)])

    rviz_node1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', LaunchConfiguration('rvizconfig')],
        arguments=['-d', str(moveit_rviz_config_path)],
        condition=IfCondition(LaunchConfiguration('moveit_rviz'))
    )

    rviz_node2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', LaunchConfiguration('rvizconfig')],
        arguments=['-d', str(default_rviz_config_path)],
        condition=UnlessCondition(LaunchConfiguration('moveit_rviz'))
    )

    # msg_exit = LogInfo(msg=('Dirección del paquete seleccionado: ', LaunchConfiguration('moveit_rviz')))

    return LaunchDescription([
        moveit_rviz_arg,
        # rviz_arg,
        rviz_node1,
        rviz_node2,
        # msg_exit,
    ])
