from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', default_value='true',
        description='for synchronization'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='visualize',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        sim_time_arg,
        rviz_node
    ])