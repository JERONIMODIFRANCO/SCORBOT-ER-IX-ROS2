#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sbot_gazebo = get_package_share_directory('sbot_gazebo')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "sbot_description"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in sbot_gazebo package
    gazebo_models_path = os.path.join(pkg_sbot_gazebo, 'models')
    # os.environ["IGN_GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'IGN_GAZEBO_MODEL_PATH' in os.environ:
        os.environ['IGN_GAZEBO_MODEL_PATH'] =  os.environ['IGN_GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['IGN_GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH' in os.environ:
        os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = install_dir + '/lib'

    

    print("GAZEBO MODELS PATH=="+str(os.environ["IGN_GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/opt/ros/humble/share/ros_ign_gazebo', 'launch', 'ign_gazebo.launch.py'),
        )
    )    

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_sbot_gazebo, 'worlds', 'empty'), ''],
          description='SDF world file'),
        gazebo
    ])