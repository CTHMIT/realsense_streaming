from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable,
    DeclareLaunchArgument, LogInfo
)
from launch.substitutions import (
    PathJoinSubstitution, LaunchConfiguration
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Robot namespace'
    )
    
    # DDS
    cyclonedds_config = PathJoinSubstitution([
        FindPackageShare('server_bringup'),
        'config', 'dds', 'cyclonedds_server.xml'
    ])
    
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '161')
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    set_dds_config = SetEnvironmentVariable('CYCLONEDDS_URI',
        ['file://', cyclonedds_config])
    
    # VSLAM
    vslam_config = PathJoinSubstitution([
        FindPackageShare('server_perception'),
        'config', 'vslam', 'vslam_params.yaml'
    ])
    
    # Nvblox
    nvblox_config = PathJoinSubstitution([
        FindPackageShare('server_perception'),
        'config', 'nvblox', 'nvblox_realsense.yaml'
    ])
    
    # Container
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace=LaunchConfiguration('robot_namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # H264 - Color
            ComposableNode(
                package='isaac_ros_h264_decoder',
                plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
                name='decoder_color',
                parameters=[{
                    'input_width': 848,
                    'input_height': 480,
                }],
                remappings=[
                    ('image_compressed', '/camera/color/image_compressed/h264'),
                    ('image_raw', '/server/color/image_raw'),
                ]
            ),
            
            # H264 - Infra1
            ComposableNode(
                package='isaac_ros_h264_decoder',
                plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
                name='decoder_infra1',
                parameters=[{
                    'input_width': 848,
                    'input_height': 480,
                }],
                remappings=[
                    ('image_compressed', '/camera/infra1/image_compressed/h264'),
                    ('image_raw', '/server/infra1/image_raw'),
                ]
            ),
            
            # H264 - Infra2
            ComposableNode(
                package='isaac_ros_h264_decoder',
                plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
                name='decoder_infra2',
                parameters=[{
                    'input_width': 848,
                    'input_height': 480,
                }],
                remappings=[
                    ('image_compressed', '/camera/infra2/image_compressed/h264'),
                    ('image_raw', '/server/infra2/image_raw'),
                ]
            ),
            
            # Visual SLAM
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[vslam_config],
                remappings=[
                    ('visual_slam/image_0', '/server/infra1/image_raw'),
                    ('visual_slam/camera_info_0', '/server/infra1/camera_info'),
                    ('visual_slam/image_1', '/server/infra2/image_raw'),
                    ('visual_slam/camera_info_1', '/server/infra2/camera_info'),
                    ('visual_slam/imu', '/camera/imu'),
                ]
            ),
            
            # Nvblox
            ComposableNode(
                package='nvblox_ros',
                plugin='nvblox::NvbloxNode',
                name='nvblox_node',
                parameters=[
                    nvblox_config,
                    {'depth_image_transport': 'compressedDepth'},
                ],
                remappings=[
                    ('depth/image', '/camera/depth/compressed'),
                    ('depth/camera_info', '/camera/depth/camera_info'),
                    ('color/image', '/server/color/image_raw'),
                    ('color/camera_info', '/server/color/camera_info'),
                    ('pose', '/visual_slam/tracking/odometry'),
                ]
            ),
        ],
        output='screen'
    )
    
    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('server_bringup'),
                'launch', 'server_visualization.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        robot_namespace_arg,
        set_domain_id,
        set_rmw,
        set_dds_config,
        LogInfo(msg=['Start the server processing node, namespace: ',
                     LaunchConfiguration('robot_namespace')]),
        perception_container,
        rviz,
    ])