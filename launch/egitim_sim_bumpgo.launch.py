import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    obstacle_distance_arg = DeclareLaunchArgument(
        'obstacle_distance',
        default_value='0.5',
        description='Engel eşik mesafesi (metre)'
    )

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.2',
        description='İleri yönlü hız (m/s)'
    )

    backing_time_arg = DeclareLaunchArgument(
        'backing_time',
        default_value='2.0',
        description='Geri gitme süresi (saniye)'
    )

    turning_time_arg = DeclareLaunchArgument(
        'turning_time',
        default_value='2.0',
        description='Dönme süresi (saniye)'
    )

    # --- Sim launch argümanı (br2_tiago ile uyumlu) ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='house',
        description='Gazebo dünya adı',
    )

    # --- br2_tiago sim.launch.py dahil et ---
    br2_tiago_dir = get_package_share_directory('br2_tiago')

    sim_lauch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(br2_tiago_dir, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    egitim_bumpgo_cmd = Node(
        package='ros2_egitim_bumpgo_cpp',
        executable='egitim_bumpgo',
        output='screen',

        parameters=[{
            'use_sim_time': True,
            'obstacle_distance' : LaunchConfiguration('obstacle_distance'),
            'linear_speed' : LaunchConfiguration('linear_speed'),
            'backing_time' : LaunchConfiguration('backing_time'),
            'turning_time' : LaunchConfiguration('turning_time'),
        }]
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(obstacle_distance_arg)
    ld.add_action(linear_speed_arg)
    ld.add_action(backing_time_arg)
    ld.add_action(turning_time_arg)
    ld.add_action(sim_lauch)  # sim + Gazebo + Tiago
    ld.add_action(egitim_bumpgo_cmd)

    return ld