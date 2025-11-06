import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # RViz 設定檔路徑
    # 注意：您還需要提供這個 .rviz 檔案
    rviz_config_file = os.path.join(
        get_package_share_directory('server_bringup'),
        'rviz',
        'server.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])