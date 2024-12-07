import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_name = 'robot.xacro'

    share_dir = get_package_share_directory('line_follower')

    urdf_file_path = os.path.join(share_dir, 'urdf', urdf_name)

    robot_description = ParameterValue(Command(['xacro ', urdf_file_path]), value_type=str)

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_pub',
        output='screen',
        arguments=[urdf_file_path]
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_pub',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_pub_gui',
        output='screen',
        arguments=[urdf_file_path]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen'
    )

    return LaunchDescription([
        joint_state_pub,
        robot_state_pub,
        joint_state_pub_gui,
        rviz_node
    ])