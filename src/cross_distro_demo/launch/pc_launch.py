from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cross_distro_demo',
            executable='pc_publisher',
            name='pc_publisher',
            output='screen',
            parameters=['/home/sergio/ros2_ws/ros2_ws_wsl2_RasPI/src/cross_distro_demo/config/params.yaml']
        )
    ])