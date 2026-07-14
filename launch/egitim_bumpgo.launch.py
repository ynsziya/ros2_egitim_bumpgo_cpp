from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():


    egitim_bumpgo_cmd = Node(
        package='ros2_egitim_bumpgo_cpp',
        executable='egitim_bumpgo',
        output='screen',

        parameters=[{
            'obstacle_distance' : LaunchConfiguration('obstacle_distance'),
            'front_angle' : LaunchConfiguration('front_angle'),
            'linear_speed' : LaunchConfiguration('linear_speed'),
            'angular_speed' : LaunchConfiguration('angular_speed'),
            'backup_speed' : LaunchConfiguration('backup_speed'),
            'backing_time' : LaunchConfiguration('backing_time'),
            'turning_time' : LaunchConfiguration('turning_time'),
        }]
    )

    ld = LaunchDescription()
    ld.add_action(egitim_bumpgo_cmd)
    return ld