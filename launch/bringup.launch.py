from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取ROS包的路径
    bringup_dir = get_package_share_directory('nav_bringup')
    nav2_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')

    # 声明启动文件需要的参数
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false', description='是否使用模拟时间'),
        DeclareLaunchArgument('lio_rviz', default_value='false', description='是否在RViz中可视化FAST_LIO'),
        DeclareLaunchArgument('nav_rviz', default_value='true', description='是否在RViz中可视化导航功能'),
        DeclareLaunchArgument('world', default_value='328', description='地图文件的名称（不包含后缀）'),
        DeclareLaunchArgument('mode', default_value='nav', description='运行模式,支持导航(nav)或建图(mapping)'),
    ]

    # 加载机器人模型描述文件
    robot_description = PathJoinSubstitution([
        bringup_dir, 'urdf', 'sentry_robot_real.xacro'
    ])
    state_pub_node = Node(
        package='robot_state_publisher',  # 用于发布机器人状态的ROS节点
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,  # 机器人描述文件
                'use_sim_time': LaunchConfiguration('use_sim_time')  # 是否使用模拟时间
            }
        ]
    )

    # LIO功能组（包括静态TF和RViz显示）
    lio_group = GroupAction([
        # 静态TF，用于发布odom到lidar_odom的转换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '--frame-id', 'odom', '--child-frame-id', 'lidar_odom']
        ),
        # 可选：是否显示LIO的RViz配置
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([bringup_dir, 'rviz', 'fastlio.rviz'])],
            condition=IfCondition(LaunchConfiguration('lio_rviz'))
        )
    ])

    # 导入导航功能的启动文件
    nav_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'bringup_rm_navigation.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # 组合整个Launch描述
    ld = LaunchDescription(declare_args)  # 添加参数声明
    ld.add_action(state_pub_node)        # 添加机器人状态发布节点
    ld.add_action(lio_group)            # 添加LIO功能组
    ld.add_action(nav_group)            # 添加导航功能组

    return ld

