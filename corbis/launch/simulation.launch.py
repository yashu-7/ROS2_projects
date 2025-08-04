import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    share_pkg = get_package_share_directory('corbis')
    
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_pkg, 'launch', 'gazebo.launch.py')
        ),
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_pkg, 'launch', 'rsp.launch.py')
        ),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_pkg, 'launch', 'visualize.launch.py')
        ),
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_pkg, 'launch', 'mapping.launch.py')
        ),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_pkg, 'launch', 'navigation.launch.py')
        ),
    )

    map_saver_node = Node(
        package='corbis',
        executable='save_map',
        name='map_saver',
        output='screen'
    )

    coordinates_saver_node = Node(
        package='corbis',
        executable='save_coords',
        name='coordinates_saver',
        output='screen'
    )

    nodes_launch = ExecuteProcess(
        cmd=['/home/yaswanth/ros2_ws/src/corbis/scripts/nodes.launch.sh'],
        name='Run_Scripts',
        output='screen',
        shell=True,  # Use shell=True to run the bash script
    )

    navigation_node = Node(
        package='corbis',
        executable='navigation',
        name='navigate',
        output='screen'
    )

    return LaunchDescription([
        rsp_launch,
        gz_launch,
        rviz_launch,
        mapping_launch,
        nodes_launch,
        map_saver_node,
        coordinates_saver_node,
        navigation_launch,
        navigation_node
    ])