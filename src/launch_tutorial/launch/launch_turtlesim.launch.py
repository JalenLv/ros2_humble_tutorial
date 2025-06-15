from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('launch_tutorial'), 'launch'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'turtlesim_world_1.launch.py'])
        ),
        GroupAction(
            actions=[
                PushRosNamespace('turtlesim2'),
                IncludeLaunchDescription(
                    PathJoinSubstitution([launch_dir, 'turtlesim_world_2.launch.py'])
                ),
            ],
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'turtlesim_world_3.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'broadcaster_listener.launch.py']),
            launch_arguments={
                'target_frame': 'turtle1'
            }.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'mimic.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'fixed_broadcaster.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'turtlesim_rviz.launch.py'])
        ),
    ])
