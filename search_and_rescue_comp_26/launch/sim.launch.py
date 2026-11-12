import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


challenge_pkg = 'multi_robot_challenge_23'

START_POSITIONS = {
    1: [(0.0, -1.0), (0.0, 1.0), (-0.6, -1.0), (-0.6, 1.0), (0.6, 1.0)],
    2: [(-1.0, 1.0), (-1.0, -1.0), (-1.6, 1.0), (-1.6, -1.0), (-0.4, -1.0)],
    3: [(-1.0, 0.5), (-1.0, -1.5), (-1.6, 0.5), (-1.6, -1.5), (-0.4, -1.5)],
    4: [(-0.5, 0.7), (-0.5, -1.3), (-1.1, 0.7), (-1.1, -1.3), (0.1, -1.3)],
    5: [(0.0, -0.5), (0.0, -2.5), (-0.6, -0.5), (-0.6, -2.5), (0.6, -2.5)],
}


def launch_setup(context):
    world = int(LaunchConfiguration('world').perform(context))
    robots = int(LaunchConfiguration('robots').perform(context))
    if world not in START_POSITIONS or not 1 <= robots <= 5:
        raise RuntimeError('world must be 1-5 and robots 1-5 (the scoring node only tracks tb3_0 - tb3_4)')

    share = get_package_share_directory(challenge_pkg)
    world_file = os.path.join(get_package_share_directory('search_and_rescue_comp_26'), 'worlds', f'dat160_w{world}.world')
    map_file = os.path.join(share, 'maps', f'map_dat160_w{world}.yaml')
    rviz_file = os.path.join(get_package_share_directory('search_and_rescue_comp_26'), 'rviz', 'team.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_file}.items()),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': map_file, 'topic_name': 'map', 'frame_id': 'map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             output='screen',
             parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': ['map_server']}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_file],
             parameters=[{'use_sim_time': use_sim_time}], output='log'),
    ]

    for i in range(robots):
        x, y = START_POSITIONS[world][i]
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('search_and_rescue_comp_26'), 'launch', 'spawn_robot.launch.py')),
            launch_arguments={'namespace': f'tb3_{i}', 'x': str(x), 'y': str(y), 'yaw': '0.0'}.items()))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='1', description='Competition map 1-5'),
        DeclareLaunchArgument('robots', default_value='4', description='Number of robots 1-5'),
        OpaqueFunction(function=launch_setup),
    ])
