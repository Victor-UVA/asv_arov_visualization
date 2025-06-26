from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='asv_arov_visualization').find('asv_arov_visualization')
    gz_pkg_share = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'tank_world.sdf')
    rviz_path = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    gazebo_path = os.path.join(gz_pkg_share, 'launch', 'gz_sim.launch.py')

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_path]),
        launch_arguments={'gz_args': f"-r -v 4 {world_path}"}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", rviz_path],
        output="screen"
    )

    bridge_asv_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_asv_amcl',
        arguments=[
            '/asv/amcl_pose@geometry_msgs/msg/PoseWithCovarianceStamped@gz.msgs.PoseWithCovariance'
        ],
        output='screen'
    )

    bridge_arov_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_arov_amcl',
        arguments=[
            '/arov/amcl_pose@geometry_msgs/msg/PoseWithCovarianceStamped@gz.msgs.PoseWithCovariance'
        ],
        output='screen'
    )

    return LaunchDescription([gazebo_sim, rviz_node, bridge_asv_node, bridge_arov_node])
