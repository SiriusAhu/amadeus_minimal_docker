from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='web_controller',
            executable='web_api_server',
            name='web_api_controller_node',
            output='screen'
        )
    ])