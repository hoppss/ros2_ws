import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from launch.actions import (DeclareLaunchArgument, GroupAction, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable)
def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    #stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    #SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # stdout_use_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1')
    # stdout_buffer_envvar=SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    #stdout_buffer_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # stdout_use_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1')
    # stdout_buffer_envvar=SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0')

    start_cmd = Node(
        package='rcl_cpp_tutorials',
        executable='pub',
        name='pub',
        namespace=namespace,
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # ld.add_action(stdout_use_envvar)
    # ld.add_action(stdout_buffer_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)

    # Add any conditioned actions
    ld.add_action(start_cmd)

    return ld
