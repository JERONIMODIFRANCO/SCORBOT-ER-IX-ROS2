# importamos utilidades 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import  LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    #Declaracion de argumentos de joint state publisher node 
    jsp_arg = DeclareLaunchArgument(name='joint_state_publisher', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable Joint State Publisher')

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('joint_state_publisher'))
    )

    return LaunchDescription([
        jsp_arg,
        joint_state_publisher_node,
    ])
