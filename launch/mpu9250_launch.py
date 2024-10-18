from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value= 'mpu9250_params.yaml',
            description= 'YAML file with configuration params'
        ),
        Node(
            package= 'mpu9250_publisher',
            executable= 'mpu9250',
            name='mpu9250_publisher',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
    ])