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

from ament_index_python.packages import get_package_share_path

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("scorbot", package_name="sbot_moveit").to_moveit_configs()
    # gazebo_path = get_package_share_path('sbot_gazebo')
    # description_path = get_package_share_path('sbot_description')
    
    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    # ld.add_action(DeclareBooleanLaunchArg("sim_gazebo", default_value=False))

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                # str(description_path / "launch/rsp.launch.py")
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
            # condition=IfCondition(LaunchConfiguration("sim_gazebo")),
        )
    )

    # if("sim_gazebo" == True):
    #     #Given the tf for the robot, spawn it in gazebo
    #     ld.add_action(
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 str( gazebo_path / "launch/spawn_robot.launch.py")
    #             ),
    #         )
    #     )


    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    ld.add_action(DeclareBooleanLaunchArg("moveit_rviz", default_value=True))
    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                # str(description_path / "launch/rviz.launch.py")
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("moveit_rviz")),
        )
    )

    # # If database loading was enabled, start mongodb as well
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             str(moveit_config.package_path / "launch/warehouse_db.launch.py")
    #         ),
    #         condition=IfCondition(LaunchConfiguration("db")),
    #     )
    # )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    # if ("sim_gazebo" == True):
    #     ld.add_action(DeclareBooleanLaunchArg("no_sim_gazebo", default_value=False))
    # else:
    #     ld.add_action(DeclareBooleanLaunchArg("no_sim_gazebo", default_value=True))

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
            # condition=IfCondition(LaunchConfiguration("sim_gazebo","no_sim_gazebo")),
        )
    )

    return ld