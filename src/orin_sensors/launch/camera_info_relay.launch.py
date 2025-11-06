from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_color_info',
            arguments=[
                '/camera/color/camera_info',
                '/camera/color/camera_info'
            ]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_infra1_info',
            arguments=[
                '/camera/infra1/camera_info',
                '/camera/infra1/camera_info'
            ]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_infra2_info',
            arguments=[
                '/camera/infra2/camera_info',
                '/camera/infra2/camera_info'
            ]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_depth_info',
            arguments=[
                '/camera/depth/camera_info',
                '/camera/depth/camera_info'
            ]
        ),
    ])