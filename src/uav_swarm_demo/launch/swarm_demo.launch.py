from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('uav_swarm_demo')

    return LaunchDescription([
        # ── Launch arguments ──────────────────────────────────────────────
        DeclareLaunchArgument('start_x',         default_value='0'),
        DeclareLaunchArgument('start_y',         default_value='0'),
        DeclareLaunchArgument('goal_x',          default_value='19'),
        DeclareLaunchArgument('goal_y',          default_value='19'),
        DeclareLaunchArgument('grid_resolution', default_value='1.0'),
        DeclareLaunchArgument('altitude',        default_value='5.0'),
        DeclareLaunchArgument('max_speed',       default_value='2.0'),
        DeclareLaunchArgument('update_rate',     default_value='10.0'),
        DeclareLaunchArgument('frame_id',        default_value='map'),

        # ── Swarm controller node ─────────────────────────────────────────
        Node(
            package='uav_swarm_demo',
            executable='swarm_controller',
            name='swarm_controller',
            output='screen',
            parameters=[{
                'start_x':         LaunchConfiguration('start_x'),
                'start_y':         LaunchConfiguration('start_y'),
                'goal_x':          LaunchConfiguration('goal_x'),
                'goal_y':          LaunchConfiguration('goal_y'),
                'grid_resolution': LaunchConfiguration('grid_resolution'),
                'altitude':        LaunchConfiguration('altitude'),
                'max_speed':       LaunchConfiguration('max_speed'),
                'update_rate':     LaunchConfiguration('update_rate'),
                'frame_id':        LaunchConfiguration('frame_id'),
            }],
        ),

        # ── RViz ─────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg, 'config', 'swarm_demo.rviz'])],
            output='screen',
        ),
    ])
