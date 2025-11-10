from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable, 
    DeclareLaunchArgument, LogInfo
)
from launch.substitutions import (
    PathJoinSubstitution, LaunchConfiguration, 
    EnvironmentVariable
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',  # Empty namespace for global topics
        description='Robot namespace'
    )
    
    # DDS
    cyclonedds_config = PathJoinSubstitution([
        FindPackageShare('client'),
        'bringup', 'config', 'dds', 'cyclonedds_orin.xml'
    ])
    
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '161')
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    set_dds_config = SetEnvironmentVariable('CYCLONEDDS_URI', 
        ['file://', cyclonedds_config])
    
    # RealSense
    realsense_config = PathJoinSubstitution([
        FindPackageShare('client'),
        'sensors', 'config', 'realsense_params.yaml'
    ])
    
    # NITROS Container
    isaac_container = ComposableNodeContainer(
        name='isaac_container',
        namespace=LaunchConfiguration('robot_namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # RealSense Camera
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera',
                parameters=[realsense_config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            
            # H264 Encoder - Color
            ComposableNode(
                package='isaac_ros_h264_encoder',
                plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                name='encoder_color',
                parameters=[{
                    'input_width': 848,
                    'input_height': 480,
                    'config': 'pframe',
                    'profile': 'main',
                    'qp': 20,
                    'hw_preset_type': 'medium',
                    'iframe_interval': 30,
                }],
                remappings=[
                    ('image_raw', '/camera/color/image_raw'),
                    ('image_compressed', '/camera/color/image_compressed/h264'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            
            # H264 Encoder - Infra1
            ComposableNode(
                package='isaac_ros_h264_encoder',
                plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                name='encoder_infra1',
                parameters=[{
                    'input_width': 848,
                    'input_height': 480,
                    'config': 'pframe',
                    'profile': 'main',
                    'qp': 20,
                    'hw_preset_type': 'medium',
                    'iframe_interval': 30,
                }],
                remappings=[
                    ('image_raw', '/camera/infra1/image_rect_raw'),
                    ('image_compressed', '/camera/infra1/image_compressed/h264'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            
            # H264 Encoder - Infra2
            ComposableNode(
                package='isaac_ros_h264_encoder',
                plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                name='encoder_infra2',
                parameters=[{
                    'input_width': 848,
                    'input_height': 480,
                    'config': 'pframe',
                    'profile': 'main',
                    'qp': 20,
                    'hw_preset_type': 'medium',
                    'iframe_interval': 30,
                }],
                remappings=[
                    ('image_raw', '/camera/infra2/image_rect_raw'),
                    ('image_compressed', '/camera/infra2/image_compressed/h264'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen'
    )
    
    depth_compressor = Node(
        package='image_transport',
        executable='republish',
        name='depth_compressor',
        namespace=LaunchConfiguration('robot_namespace'),
        arguments=[
            'raw',
            'compressedDepth',
            '--ros-args',
            '--remap', 'in:=/camera/aligned_depth_to_color/image_raw',
            '--remap', 'out/compressedDepth:=/camera/depth/compressedDepth',
            '--param', 'compressedDepth.format:=png',
            '--param', 'compressedDepth.png_level:=3',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        robot_namespace_arg,
        set_domain_id,
        set_rmw,
        set_dds_config,
        LogInfo(msg=['Starting Orin client node, namespace: ', 
                     LaunchConfiguration('robot_namespace')]),
        isaac_container,
        depth_compressor,
    ])