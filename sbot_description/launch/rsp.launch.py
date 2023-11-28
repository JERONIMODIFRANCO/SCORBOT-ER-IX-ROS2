from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    description_package_path = get_package_share_path('sbot_description')
    gazebo_package_path = get_package_share_path('sbot_gazebo')
    default_model_path = description_package_path / 'urdf/scorbot.urdf.xacro'
    gazebo_model_path = gazebo_package_path / 'config/sbot.urdf.xacro'

    # path_model_arg = DeclareLaunchArgument(name='path_model', default_value=str(default_model_path),
    #                                   description='select the path to the model',
    #                                   choices=[str(default_model_path),str(gazebo_model_path)])


    # sim_gazebo_arg = DeclareLaunchArgument(name='sim_gazebo', default_value=str(False),
    #                                   description='select sim gazebo')
    # if(sim_gazebo_arg == 'True'):
    #     # Si es verdadero se utiliza la descripcion ampliada        
    #     default_package_path = get_package_share_path('sbot_gazebo')
    #     default_model_path = default_package_path / 'config/sbot.urdf.xacro'
    # else:
    #     # Si es falso se utiliza la descripcion base
    #     default_package_path = get_package_share_path('sbot_description')
    #     default_model_path = default_package_path / 'urdf/scorbot.urdf.xacro'

    # model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
    #                                   description='Absolute path to robot urdf file')

    # robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('path_model')]),
    #                                    value_type=str)
    robot_description = ParameterValue(Command(['xacro ', str(gazebo_model_path)]),
                                       value_type=str)
   
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output="screen"
    )

    # msg_exit = LogInfo(msg=('Dirección del paquete seleccionado: ', LaunchConfiguration('path_model')))

    return LaunchDescription([
        # path_model_arg,
        # msg_exit,
        # model_arg,
        robot_state_publisher_node,
    ])
