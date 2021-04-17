from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(package='lifecyclee', executable='lifecycle_talker',
                      name='lc_talker', output='screen'),
        Node(package='lifecyclee', executable='lifecycle_listener', output='screen'),
        Node(package='lifecyclee', executable='lifecycle_service_client', output='screen')
    ])