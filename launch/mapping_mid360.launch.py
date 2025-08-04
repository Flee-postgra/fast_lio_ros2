import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch argument to enable RViz
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Enable RViz visualization'
    )

    # Locate package share directories
    pkg_fast_lio = get_package_share_directory('fast_lio')
    config_file = os.path.join(pkg_fast_lio, 'config', 'mid360_g1.yaml')  # fileciteturn0file0
    rviz_config = os.path.join(pkg_fast_lio, 'rviz_cfg', 'loam_livox.rviz')

    # FAST_LIO node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio_node',
        output='screen',
        parameters=[
            config_file,
            {
                'feature_extract_enable': False,
                'point_filter_num':       2,
                'max_iteration':          3,
                'filter_size_surf':       0.5,
                'filter_size_map':        0.8,
                'cube_side_length':    1000.0,
                'runtime_pos_log_enable': False,
                'use_sim_time':           False,

                # 预处理
                'preprocess.lidar_type':        1,               # 1 = Livox CustomMsg
                'preprocess.scan_line':         4,               # Mid-360 扫描线数
                'preprocess.blind':             0.5,             # 盲区设置
                'preprocess.pointcloud_topic': '/livox/lidar',   # 指定订阅话题

                # 建图
                'mapping.extrinsic_est_en': False,  # 暂时关闭外参估计
                'mapping.scan_period':      0.1,
                'mapping.gyr_cov':          0.01,   # 降低陀螺仪噪声协方差
                'mapping.acc_cov':          0.01,   # 降低加速度计噪声协方差

                # 发布
                'publish.path_en': True,     # ← 这里现在有逗号
            }
        ]
    )

    # Optional RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
        prefix='nice',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        fast_lio_node,
        rviz_node
    ])

