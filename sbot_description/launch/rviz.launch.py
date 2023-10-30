from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_package_path = get_package_share_path('sbot_description')
    default_model_path = default_package_path / 'urdf/scorbot.urdf.xacro'
    default_rviz_config_path = default_package_path / 'rviz/config.rviz'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    jsp_arg = DeclareLaunchArgument(name='joint_state_publisher', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable Joint State Publisher')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
   
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    return LaunchDescription([
        model_arg,
        rviz_arg,
        jsp_arg,
        robot_state_publisher_node,
        rviz_node
    ])
