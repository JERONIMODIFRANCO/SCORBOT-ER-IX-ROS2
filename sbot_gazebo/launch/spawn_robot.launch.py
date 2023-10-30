#!/usr/bin/python3
# -*- coding: utf-8 -*-
from ament_index_python.packages import get_package_share_path

import random
from launch_ros.actions import Node
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.parameter_descriptions import ParameterValue

# this is the function launch  system will look for

def generate_launch_description():

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.0]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "scorbot"

    entity_name = robot_base_name+"-"+str(int(random.random()*100000))

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='ros_ign_gazebo',
        # package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_robot,
        ]
    )