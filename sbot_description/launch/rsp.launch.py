from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Path descripcion por defecto (solo rviz)
    description_package_path = get_package_share_path('sbot_description') 
    default_model_path = description_package_path / 'urdf/scorbot.urdf.xacro'
    model_arg_base = DeclareLaunchArgument(name='model_base', default_value=str(default_model_path),
                                      description='Absolute path to robot base urdf file')
    robot_description_base = ParameterValue(Command(['xacro ', LaunchConfiguration('model_base')]),
                                       value_type=str)
    
    # Path descripcion con configuracion para gazebo
    gazebo_package_path = get_package_share_path('sbot_gazebo')
    gazebo_model_path = gazebo_package_path / 'config/scorbot_gz.urdf.xacro'
    model_arg_gz = DeclareLaunchArgument(name='model_gz', default_value=str(gazebo_model_path),
                                      description='Absolute path to robot gz urdf file')
    robot_description_gz = ParameterValue(Command(['xacro ', LaunchConfiguration('model_gz')]),
                                       value_type=str)
    # Argumento de seleccion de modelo 
    sim_gazebo_arg = DeclareLaunchArgument(name='sim_gazebo', default_value='true', choices=['true', 'false'],
                                      description='Flag to select robot description to simulate in gazebo or keeps the base description')

    # Nodo del robot state publisher en caso de seleccion de simulacion con gazebo
    robot_state_publisher_node_gz = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition = IfCondition(LaunchConfiguration('sim_gazebo')),
        name='robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_gz}],
        output="screen"
    )

    # Nodo del robot state publisher en caso de seleccion de no simulacion 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition = UnlessCondition(LaunchConfiguration('sim_gazebo')),
        name='robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_base}],
        output="screen"
    )

    # msg_exit = LogInfo(msg=('Dirección del paquete seleccionado: ', LaunchConfiguration('path_model')))

    return LaunchDescription([
        sim_gazebo_arg,
        model_arg_base,
        model_arg_gz,
        robot_state_publisher_node_gz,
        robot_state_publisher_node,
    ])
