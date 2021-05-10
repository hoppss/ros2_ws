
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    # Declare the launch arguments

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='ns1',
            description='Top-level namespace'),

        LifecycleNode(package='lifecyclee', executable='lifecycle_talker',
                      name='lc_talker', namespace=namespace, output='screen'),
        Node(package='lifecyclee', executable='lifecycle_listener', output='screen', namespace=namespace),
        Node(package='lifecyclee', executable='lifecycle_service_client', output='screen', namespace=namespace)
    ])