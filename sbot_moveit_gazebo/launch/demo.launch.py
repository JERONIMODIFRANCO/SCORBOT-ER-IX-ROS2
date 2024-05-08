# Launch file to dynamically control the robot in RViz.
# Para mayor claridad, verificar:
# /opt/ros/humble/lib/python3.10/site-packages/moveit_configs_utils/launches.py
# O los demás archivos de este paquete

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("scorbot", package_name="sbot_moveit_gazebo").to_moveit_configs()
    return generate_demo_launch(moveit_config)
