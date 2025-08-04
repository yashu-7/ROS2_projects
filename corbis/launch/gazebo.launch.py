import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_name = 'corbis'

    package_path = get_package_share_directory('corbis')
    
    world_path = os.path.join(package_path, 'world', 'world.sdf')

    declare_gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value=f'{world_path}',
        description='Gazebo world file to load'
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', f'{robot_name}',
                   '-z', '0.1'],
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    bridge_param_path = os.path.join(package_path, 'config', 'ros_gz_bridge.yaml')

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p', f'config_file:={bridge_param_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    gz_img_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/camera/image'
        ],
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'camera.image.compressed.jpeg_quality':75},
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(package_path, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        declare_gz_args,
        gz_launch,
        spawn_robot,
        gz_bridge_node,
        gz_img_bridge,
        ekf_node
    ])