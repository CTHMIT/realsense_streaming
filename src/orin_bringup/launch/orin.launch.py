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
        FindPackageShare('orin_bringup'),
        'config', 'dds', 'cyclonedds_orin.xml'
    ])
    
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '161')
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    set_dds_config = SetEnvironmentVariable('CYCLONEDDS_URI', 
        ['file://', cyclonedds_config])
    
    # RealSense
    realsense_config = PathJoinSubstitution([
        FindPackageShare('orin_sensors'),
        'config', 'realsense_params.yaml'
    ])
    
    # NITROS
    isaac_container = ComposableNodeContainer(
        name='isaac_container',
        namespace=LaunchConfiguration('robot_namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # RealSense
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera',
                parameters=[realsense_config],
            ),
            
            # H264 - Color
            ComposableNode(
                package='isaac_ros_h264_encoder',
                plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                name='encoder_color',
                parameters=[{
                    'input_width': 848,
                    'input_height': 480,
                    'config': 'pframe',
                }],
                remappings=[
                    ('image_raw', '/camera/color/image_raw'),
                    ('image_compressed', '/camera/color/image_compressed/h264'),
                ]
            ),

                        
            # H264 - Infra1
            ComposableNode(
                package='isaac_ros_h264_encoder',
                plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                name='encoder_infra1',
                parameters=[{
                    'input_width': 848,
                    'input_height': 480,
                    'config': 'pframe',
                }],
                remappings=[
                    ('image_raw', '/camera/infra1/image_rect_raw'),
                    ('image_compressed', '/camera/infra1/image_compressed/h264'),
                ]
            ),


            
            # H264 - Infra2
            ComposableNode(
                package='isaac_ros_h264_encoder',
                plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                name='encoder_infra2',
                parameters=[{
                    'input_width': 848,
                    'input_height': 480,
                    'config': 'pframe',
                }],
                remappings=[
                    ('image_raw', '/camera/infra2/image_rect_raw'),
                    ('image_compressed', '/camera/infra2/image_compressed/h264'),
                ]
            ),


            
            # zdepth_image_transport
            ComposableNode(
                package='image_transport',
                plugin='image_transport::Publisher',
                name='depth_publisher',
                parameters=[{
                    'transport': 'compressedDepth',
                    'depth_image_transport.compression_format': 'zstd',
                }],
                remappings=[
                    ('in', '/camera/aligned_depth_to_color/image_raw'),
                    ('out/compressed', '/camera/depth/compressed'),
                ]
            ),

        ],
        output='screen'
    )
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch', 'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': LaunchConfiguration('robot_namespace')
        }.items()
    )
    
    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_diagnostics'),
                'launch', 'diagnostics_aggregator.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        robot_namespace_arg,
        set_domain_id,
        set_rmw,
        set_dds_config,
        LogInfo(msg=['Start the Orin client node, namespace: ', 
                     LaunchConfiguration('robot_namespace')]),
        isaac_container,
        robot_state_publisher,
        diagnostics,
    ])