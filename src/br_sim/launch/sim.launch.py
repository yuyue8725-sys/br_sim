import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('br_sim')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_share, 'worlds', 'robocon.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'br.urdf')

    # 读取机器人 URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Gazebo Classic 11 的 mesh URI 处理器不支持 package:// 协议：
    # 它会静默失败（不报错但 mesh 不显示），导致 Gazebo 里看不到机器人。
    # 这里把 package:// 统一替换为 file:// 绝对路径，Gazebo 和 RViz2 都能正确解析。
    robot_description = robot_description.replace(
        'package://br_sim', 'file://' + pkg_share)

    # 启动 Gazebo Classic（gazebo_ros 官方 launch：
    # 自动加载 libgazebo_ros_init.so（发布 /clock）和 libgazebo_ros_factory.so（提供 /spawn_entity），
    # 自动设置 GAZEBO_MODEL_PATH / GAZEBO_PLUGIN_PATH / GAZEBO_RESOURCE_PATH）
    # 注意：必须用 Gazebo Classic 的 gazebo.launch.py，不能用 ign gazebo（Ignition），
    # 因为 gazebo_ros 的传感器插件是为 Gazebo Classic 11 编译的。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 发布 TF（base_link -> mid360_link 等关节变换）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # 把机器人 spawn 到 Gazebo（等待 Gazebo 就绪）
    # 用 -topic 而非 -file：这样 spawn 用的是 robot_state_publisher 发布的
    # 已经过 file:// 路径替换的 URDF（mesh 才能被 Gazebo 正确加载）。
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        # 出生点必须避开场地里的障碍物！robocon_ground 在 (4.15, 0) 有个
        # r=0.135、高 0.5 m 的 Cylinder，而原出生点 (4.25, 0) 正好把它包在
        # 机器人碰撞体内部：深度穿插会让 ODE 生成不出接触，机器人直接穿过
        # 地面无限下坠（实测掉到 z=-3.3），也是之前「旋转失灵/翻车」的真凶。
        # (4.5, 2.0) 位于中心平台（|x|,|y|<=3）与围墙（|x|,|y|=5.45）之间的
        # 环形通道内，四周无障碍。
        # z 取 0.06：机器人唯一的 box 碰撞体底面在 base_link 系 -0.055，
        # 所以静止时 base_link 应停在 z≈0.055。从 0.06 落下只有 5 mm，
        # 避免高空落地冲击。
        arguments=['-entity', 'br',
                   '-topic', 'robot_description',
                   '-x', '4.5',
                   '-y', '2.0',
                   '-z', '0.06',
                   '-Y', '0.0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        TimerAction(period=5.0, actions=[spawn_entity]),
    ])

