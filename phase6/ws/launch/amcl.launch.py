from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='地図 YAML ファイルのフルパス'
        ),
        DeclareLaunchArgument(
            'init_x',
            default_value='0.0',
            description='AMCL 初期位置 x [m]'
        ),
        DeclareLaunchArgument(
            'init_y',
            default_value='0.0',
            description='AMCL 初期位置 y [m]'
        ),
        DeclareLaunchArgument(
            'init_yaw',
            default_value='0.0',
            description='AMCL 初期姿勢 yaw [rad]'
        ),

        Node(
            # 静的地図を配信する map_server
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                '/ws/config/amcl.yaml',
                {'yaml_filename': map_yaml}
            ]
        ),

        Node(
            # 静的地図ベース自己位置推定
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                '/ws/config/amcl.yaml',
                {
                    'set_initial_pose': True,
                    'always_reset_initial_pose': True,
                    'initial_pose.x': init_x,
                    'initial_pose.y': init_y,
                    'initial_pose.z': 0.0,
                    'initial_pose.yaw': init_yaw,
                }
            ]
        ),

        Node(
            # lifecycle ノードを順番に configure / activate する
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=['/ws/config/amcl.yaml']
        )
    ])