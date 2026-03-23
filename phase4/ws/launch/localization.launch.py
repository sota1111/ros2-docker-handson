from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/ws/maps/map.yaml',
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

        # EKF を 5 秒後に起動する
        # 理由: depends_on はコンテナ起動のみ保証し、Gazebo が /clock を
        # publish し始めるまでの時間を保証しない。
        # use_sim_time=true のノードは /clock が来るまで時計が進まないため
        # 少し待ってから起動してもシミュレーション時間上は問題ない。
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource('/ws/launch/ekf.launch.py')
                ),
            ]
        ),

        # AMCL (map_server + amcl + lifecycle_manager) は EKF より後に起動する
        # 理由:
        # 1. Gazebo が /clock / /tf / /scan を出し揃えるのを待つ
        # 2. EKF が odom->base_footprint TF をキャッシュに積んでから
        #    AMCL が LaserScan を処理できるようにする
        # 3. 起動タイミングが早すぎると AMCL の MessageFilter が
        #    "earlier than all the data in the transform cache" を出し続け
        #    全 scan を捨てて map->odom TF が永遠に出なくなる
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource('/ws/launch/amcl.launch.py'),
                    launch_arguments={
                        'map': map_yaml,
                        'init_x': init_x,
                        'init_y': init_y,
                        'init_yaw': init_yaw,
                    }.items()
                )
            ]
        ),
    ])
