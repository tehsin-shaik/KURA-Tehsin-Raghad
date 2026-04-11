from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_name', default_value='default'),
        DeclareLaunchArgument('publish_topic', default_value='/whistle_detection/detected'),
        DeclareLaunchArgument('cooldown_ms', default_value='1500'),
        DeclareLaunchArgument('num_threads', default_value='2'),
        Node(
            package='whistle_detector',
            executable='whistle_detector_node',
            output='screen',
            parameters=[{
                'device_name': LaunchConfiguration('device_name'),
                'publish_topic': LaunchConfiguration('publish_topic'),
                'cooldown_ms': LaunchConfiguration('cooldown_ms'),
                'num_threads': LaunchConfiguration('num_threads'),
            }]
        )
    ])
