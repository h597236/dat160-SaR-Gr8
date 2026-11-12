"""Spawnar ekstra robotar (tb3_2, tb3_3, ...) i startsona, i tillegg til dei to frå rescue_robots_wN.

    Terminal 1:  ros2 launch multi_robot_challenge_23 rescue_robots_w1.launch.py
    Terminal 2:  ros2 launch multi_robot_challenge_23 extra_robots.launch.py world:=1 n_robots:=4
    Terminal 3:  ros2 launch multi_robot_challenge_23 rescue.launch.py n_robots:=4

Dei ekstra robotane står 0.6 m bak tb3_0 og tb3_1 (minst 0.9 m til næraste vegg i alle 5 verdene).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

# startposisjonane til tb3_0 og tb3_1 i kvar verd (frå rescue_robots_wN.launch.py)
STARTS = {1: [(0.0, -1.0), (0.0, 1.0)], 2: [(-1.0, 1.0), (-1.0, -1.0)], 3: [(-1.0, 0.5), (-1.0, -1.5)],
          4: [(-0.5, 0.7), (-0.5, -1.3)], 5: [(0.0, -0.5), (0.0, -2.5)]}


def spawn_extra(context):
    world = int(context.launch_configurations['world'])
    n = int(context.launch_configurations['n_robots'])
    base = STARTS[world]
    extra_pos = [(x - 0.6, y) for (x, y) in base] + [(x + 0.6, y) for (x, y) in base]
    pkg = get_package_share_directory('multi_robot_challenge_23')
    actions = []
    for i in range(2, n):
        x, y = extra_pos[i - 2]
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'spawn_robot.launch.py')),
            launch_arguments={'namespace': f'tb3_{i}', 'x': str(x), 'y': str(y), 'yaw': '0.0',
                              'use_sim_time': 'true'}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='1'),
        DeclareLaunchArgument('n_robots', default_value='4'),
        OpaqueFunction(function=spawn_extra),
    ])