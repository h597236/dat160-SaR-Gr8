from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    robots = int(LaunchConfiguration('robots').perform(context))
    mission = LaunchConfiguration('mission').perform(context).lower() == 'true'
    scoring = LaunchConfiguration('scoring').perform(context).lower() == 'true'
    wall_check = LaunchConfiguration('use_wall_check').perform(context).lower() == 'true'
    speed = float(LaunchConfiguration('speed').perform(context))
    names = [f'tb3_{i}' for i in range(robots)]

    actions = [Node(package='search_and_rescue_comp_26', executable='marker_reporter', output='screen',
                    parameters=[{'robots': names}])]
    if scoring:
        actions.append(Node(package='scoring', executable='scoring', output='screen'))
    for name in names:
        actions.append(Node(package='search_and_rescue_comp_26', executable='navigator', namespace=name, output='log',
                            parameters=[{'speed': speed}]))
    if mission:
        actions.append(Node(package='search_and_rescue_comp_26', executable='mission', output='screen',
                            parameters=[{'robots': names, 'use_wall_check': wall_check}]))
    if wall_check:
        actions.append(Node(package='search_and_rescue_comp_26', executable='wall_check', output='log',
                            parameters=[{'robots': names}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='4'),
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('scoring', default_value='true'),
        DeclareLaunchArgument('use_wall_check', default_value='true'),
        DeclareLaunchArgument('speed', default_value='0.26'),
        OpaqueFunction(function=launch_setup),
    ])
