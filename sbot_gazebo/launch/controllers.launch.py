# launch file to proper manage of the ros2 controllers (not in use)

from launch_ros.actions import Node
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from srdfdom.srdf import SRDF
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.conditions import IfCondition, UnlessCondition

# this is the function launch  system will look for

def generate_launch_description():
    
    moveit_config = MoveItConfigsBuilder("scorbot", package_name="sbot_moveit").to_moveit_configs()

    returnList = []
   
    control_selection = DeclareLaunchArgument(name='control_type', default_value='true', choices=['true', 'false'],
                                      description='True for ros2 control, false for gz plugin control')
    returnList.append(control_selection)
    
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition = IfCondition(LaunchConfiguration('control_type')),
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
    )
    returnList.append(controller_manager)
       
    spawn_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration('control_type'))
    )
    returnList.append(spawn_controller)

    return LaunchDescription(returnList)