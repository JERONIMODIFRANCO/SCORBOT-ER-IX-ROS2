from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("scorbot", package_name="sbot_moveit").to_moveit_configs()
    return LaunchDescription([
        DeclareLaunchArgument('sim_gz', default_value='false', description='Set to true to run in Gazebo'),
        DeclareLaunchArgument('fake', default_value='false', description='Set to true to run rviz only'),
        generate_demo_launch(moveit_config)
    ])

