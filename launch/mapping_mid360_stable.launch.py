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
    config_file = os.path.join(pkg_fast_lio, 'config', 'mid360_g1.yaml')
    rviz_config = os.path.join(pkg_fast_lio, 'rviz_cfg', 'loam_livox.rviz')

    # FAST_LIO node with optimized parameters for stability
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio_node',
        output='screen',
        parameters=[
            config_file,
            {
                'feature_extract_enable': False,
                'point_filter_num':       3,       # 增加降采样减少计算负载
                'max_iteration':          2,       # 减少迭代次数提高实时性
                'filter_size_surf':       0.8,     # 进一步增大滤波器尺寸
                'filter_size_map':        1.2,     # 增大地图滤波器尺寸
                'cube_side_length':    1000.0,
                'runtime_pos_log_enable': False,
                'use_sim_time':           False,

                # 预处理 - 优化点云质量
                'preprocess.lidar_type':        1,               # 1 = Livox CustomMsg
                'preprocess.scan_line':         4,               # Mid-360 扫描线数
                'preprocess.blind':             1.0,             # 增大盲区过滤无效近距离点
                'preprocess.pointcloud_topic': '/livox/lidar',   # 指定订阅话题

                # 建图 - 保守参数提高稳定性
                'mapping.extrinsic_est_en': False,   # 关闭在线外参估计
                'mapping.scan_period':      0.1,
                'mapping.gyr_cov':          0.001,   # 进一步降低陀螺仪噪声协方差
                'mapping.acc_cov':          0.001,   # 进一步降低加速度计噪声协方差
                'mapping.det_range':        50.0,    # 减小检测范围提高稳定性
                'mapping.fov_degree':       360.0,

                # 发布
                'publish.path_en': True,
                'publish.scan_publish_en': True,
                'publish.dense_publish_en': False,  # 关闭密集点云发布减少负载
                'publish.scan_bodyframe_pub_en': False,  # 关闭body frame发布

                # PCD保存
                'pcd_save.pcd_save_en': False,
                'pcd_save.interval': -1,
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