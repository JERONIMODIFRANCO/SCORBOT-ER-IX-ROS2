from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_package_path = get_package_share_path('sbot_description')
    default_rviz_config_path = default_package_path / 'rviz/config.rviz'
    moveit_rviz_config_path = default_package_path / 'rviz/moveit.rviz'

    moveit_rviz_arg = DeclareLaunchArgument(name='moveit_rviz', default_value="False",
                                      description='select rviz with the moveit2 plugin')
    if(moveit_rviz_arg == 'True'):
        # Si es verdadero se utiliza la descripcion ampliada        
         rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(moveit_rviz_config_path),
                                     description='Absolute path to moveit_rviz config file')
   
    else:
        # Si es falso se utiliza la descripcion base
        rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')



    
   
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    return LaunchDescription([
        # moveit_rviz_arg,
        rviz_arg,
        rviz_node
    ])
