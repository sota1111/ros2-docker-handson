from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 地図 YAML のパスを launch 引数で受け取る
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='地図 YAML ファイルのフルパス'
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
            parameters=['/ws/config/amcl.yaml']
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