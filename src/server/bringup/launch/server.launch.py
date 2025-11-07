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
        FindPackageShare('server'),
        'bringup', 'config', 'dds', 'cyclonedds_server.xml'
    ])
    
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '161')
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    set_dds_config = SetEnvironmentVariable('CYCLONEDDS_URI',
        ['file://', cyclonedds_config])
    
    # VSLAM Config
    vslam_config = PathJoinSubstitution([
        FindPackageShare('server'),
        'perception', 'config', 'vslam', 'vslam_params.yaml'
    ])
    
    # Nvblox Config
    nvblox_config = PathJoinSubstitution([
        FindPackageShare('server'),
        'perception', 'config', 'nvblox', 'nvblox_realsense.yaml'
    ])
    
    # Perception Container
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace=LaunchConfiguration('robot_namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # H264 Decoder - Color
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
                    ('image_raw', '/camera/color/image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            
            # H264 Decoder - Infra1
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
                    ('image_raw', '/camera/infra1/image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            
            # H264 Decoder - Infra2
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
                    ('image_raw', '/camera/infra2/image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            
            # Depth decompressor
            ComposableNode(
                package='image_transport',
                plugin='image_transport::Republisher',
                name='depth_decompressor',
                parameters=[{
                    'in_transport': 'compressedDepth',
                    'out_transport': 'raw',
                }],
                remappings=[
                    ('in/compressedDepth', '/camera/depth/compressedDepth'),
                    ('out', '/camera/depth/image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            
            # Visual SLAM
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[vslam_config],
                remappings=[
                    ('visual_slam/image_0', '/camera/infra1/image_raw'),
                    ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),
                    ('visual_slam/image_1', '/camera/infra2/image_raw'),
                    ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),
                    ('visual_slam/imu', '/camera/imu'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            
            # Nvblox
            ComposableNode(
                package='nvblox_ros',
                plugin='nvblox::NvbloxNode',
                name='nvblox_node',
                parameters=[nvblox_config],
                remappings=[
                    ('depth/image', '/camera/depth/image_raw'),
                    ('depth/camera_info', '/camera/depth/camera_info'),
                    ('color/image', '/camera/color/image_raw'),
                    ('color/camera_info', '/camera/color/camera_info'),
                    ('pose', '/visual_slam/tracking/odometry'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen'
    )
    
    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('server'),
                'bringup', 'launch', 'camera_visualization.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        robot_namespace_arg,
        set_domain_id,
        set_rmw,
        set_dds_config,
        LogInfo(msg=['Starting server processing node, namespace: ',
                     LaunchConfiguration('robot_namespace')]),
        perception_container,
        rviz,
    ])