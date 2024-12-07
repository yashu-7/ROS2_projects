import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_name = 'line_follower'
    world_name = "track.sdf"
    package_path = get_package_share_directory('line_follower')
    urdf_file_path = os.path.join(package_path, 'urdf', f"robot.xacro")
    world_file_path = os.path.join(package_path, 'world', world_name)

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        arguments=[urdf_file_path]
    )

    robot_description = ParameterValue(Command(['xacro ', urdf_file_path]), value_type=str)


    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    declare_gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value=f'{world_file_path}',
        description='gazebo world file'
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
                   '-z', '0.0'],
        output='screen'
    )

    bridge_param_path = os.path.join(package_path, 'config', 'gz_ros_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p', f'config_file:={bridge_param_path}'
        ]
    )

    return LaunchDescription([
        joint_state_pub,
        robot_state_pub,
        declare_gz_args,
        gz_launch,
        spawn_robot,
        ros_gz_bridge,
    ])