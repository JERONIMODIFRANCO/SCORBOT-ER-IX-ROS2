# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# import xacro

# def generate_launch_description():
#     sbot_description_path = os.path.join(
#         get_package_share_directory('sbot_description'))

#     xacro_file = os.path.join(sbot_description_path,
#                               'urdf',
#                               'sbot.urdf.xacro')

#     doc = xacro.parse(open(xacro_file))
#     xacro.process_doc(doc)
#     robot_description_config = doc.toxml()
#     robot_description = {'sbot_description': robot_description_config}

#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description]
#     )

#     spawn_entity = Node(package='ros_ign_gazebo', executable='create',
#                         arguments=['-topic', 'robot_description',
#                                    '-entity', 'sbot'],
#                         output='screen')

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster",
#                    "--controller-manager", "/controller_manager"],
#     )

#     robot_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["forward_position_controller", "-c", "/controller_manager"],
#     )

#     return LaunchDescription([
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=spawn_entity,
#                 on_exit=[joint_state_broadcaster_spawner],
#             )
#         ),
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=joint_state_broadcaster_spawner,
#                 on_exit=[robot_controller_spawner],
#             )
#         ),
#         spawn_entity,
#         node_robot_state_publisher,
#     ])

# HAY QUE VER QUE OPCION ANDA MEJOR

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    robot_description_path = os.path.join(
        get_package_share_directory('sbot_description'))

    xacro_file = os.path.join(robot_description_path,
                              'urdf',
                              'sbot.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'sbot',
                   '-allow_renaming', 'true'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'effort_controllers'],
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 3 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        node_robot_state_publisher,
        ignition_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])