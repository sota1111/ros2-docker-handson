from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            # robot_localization パッケージの EKF ノード
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',

            # YAML パラメータを読み込む
            # use_sim_time を dict でも明示することで
            # /clock が来る前にノードが誤って wall clock を使うのを防ぐ
            parameters=[
                '/ws/config/ekf.yaml',
                {'use_sim_time': True},
            ],
        )
    ])