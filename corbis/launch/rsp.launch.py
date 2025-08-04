import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = 'robot.xacro'

    pkg_dir = get_package_share_directory('corbis')

    urdf_file_path = os.path.join(pkg_dir, 'urdf', urdf_file)

    robot_description = ParameterValue(Command(['xacro ', urdf_file_path]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_pub',
        output='screen',
        parameters=[{'robot_description': robot_description, 
                     'use_sim_time':True
                     }]
    )

    joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='gui_node_joint_states',
        output='screen',


    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='visualize',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        # joint_state_pub_gui,
    ])