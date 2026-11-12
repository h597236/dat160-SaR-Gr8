"""Startar heile search & rescue-systemet (alt utanom Gazebo-verda).

    Terminal 1:  ros2 launch multi_robot_challenge_23 rescue_robots_w1.launch.py
    Terminal 2:  ros2 launch multi_robot_challenge_23 rescue.launch.py
Med fleire robotar (sjå extra_robots.launch.py):  n_robots:=4
På konkurranse-PC-en køyrer arrangøren scoring sjølv:  scoring:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context):
    n = int(context.launch_configurations['n_robots'])
    robots = [f'tb3_{i}' for i in range(n)]
    common = {'use_sim_time': context.launch_configurations['use_sim_time'] == 'true'}
    v_max = float(context.launch_configurations['v_max'])
    nodes = [
        Node(package='scoring', executable='scoring', name='scoring', output='screen',
             condition=IfCondition(LaunchConfiguration('scoring'))),
        Node(package='multi_robot_challenge_23', executable='marker_reporter', name='marker_reporter',
             output='screen', parameters=[common, {'namespaces': robots}]),
        Node(package='multi_robot_challenge_23', executable='map_manager', name='map_manager',
             output='screen', parameters=[common, {'namespaces': robots}]),
        Node(package='multi_robot_challenge_23', executable='coordinator', name='coordinator',
             output='screen', parameters=[common, {'namespaces': robots}]),
    ]
    for ns in robots:
        nodes.append(Node(package='multi_robot_challenge_23', executable='robot_controller',
                          name='robot_controller', namespace=ns, output='screen',
                          parameters=[common, {'v_max': v_max}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scoring', default_value='true'),
        DeclareLaunchArgument('v_max', default_value='0.30'),
        DeclareLaunchArgument('n_robots', default_value='2'),
        OpaqueFunction(function=make_nodes),
    ])