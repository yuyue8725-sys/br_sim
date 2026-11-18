import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('br_sim')

    slam_config = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'slam.rviz')

    # SLAM 建图（online async 模式）
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config, {'use_sim_time': True}],
        output='screen'
    )

    # RViz 可视化
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([slam_toolbox, rviz2])
