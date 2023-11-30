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
    # Path de los archivos 
    default_package_path = get_package_share_path('sbot_description')
    
    # Path descripcion con configuracion base rviz
    default_rviz_config_path = default_package_path / 'rviz/config.rviz'
    rviz_arg_base = DeclareLaunchArgument(name='rviz_base', default_value=str(default_rviz_config_path),
                                      description='Absolute path to rviz base config file')    
    rviz_config_arg = LaunchConfiguration('rviz_base')

    # Path descripcion con configuracion de rviz para moveit 
    moveit_rviz_config_path = default_package_path / 'rviz/moveit.rviz'
    rviz_arg_moveit = DeclareLaunchArgument(name='rviz_moveit', default_value=str(moveit_rviz_config_path),
                                      description='Absolute path to rviz moveit config file') 
    rviz_config_moveit_arg = LaunchConfiguration('rviz_moveit')

    # Argumento de seleccion de configuracion a setear
    moveit_rviz_arg = DeclareLaunchArgument(name='rviz_with_moveit', default_value='false', 
                                    choices=['true', 'false'],
                                    description='Flag to enable RVIZ with Moveit Plugins')
    # Nodo base
    rviz_node_base = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', LaunchConfiguration('rvizconfig')],
        arguments=['-d', rviz_config_arg],
        condition=UnlessCondition(LaunchConfiguration('rviz_with_moveit'))
    )
    
    # rviz_parameters = [
    #     moveit_config.planning_pipelines,
    #     moveit_config.robot_description_kinematics,
    # ]



    # Nodo para moveit
    rviz_node_moveit = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', LaunchConfiguration('rvizconfig')],
        arguments=['-d', rviz_config_moveit_arg],
        # parameters=rviz_parameters,
        condition=IfCondition(LaunchConfiguration('rviz_with_moveit'))
    )

    # msg_exit = LogInfo(msg=('Dirección del paquete seleccionado: ', LaunchConfiguration('moveit_rviz')))

    return LaunchDescription([
        moveit_rviz_arg,
        rviz_arg_base,
        rviz_arg_moveit,
        rviz_node_base,
        rviz_node_moveit,
    ])
