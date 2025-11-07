import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    robot_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    # URDF
    pkg_path = get_package_share_directory('robot')
    urdf_file = os.path.join(pkg_path, 'description', 'urdf', 'mir_ur.urdf')
    
    if not os.path.exists(urdf_file):
        raise Exception(f"URDF file not found at: {urdf_file}")

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_namespace_arg,
        robot_state_publisher_node
    ])