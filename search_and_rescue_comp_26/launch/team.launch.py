from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    robots = int(LaunchConfiguration('robots').perform(context))
    reserves = int(LaunchConfiguration('reserves').perform(context))
    mission = LaunchConfiguration('mission').perform(context).lower() == 'true'
    wall_colour = LaunchConfiguration('use_wall_colour').perform(context).lower() == 'true'
    wait_scoring = LaunchConfiguration('wait_for_scoring').perform(context).lower() == 'true'
    speed = float(LaunchConfiguration('speed').perform(context))
    turn_speed = float(LaunchConfiguration('turn_speed').perform(context))
    face_turn = float(LaunchConfiguration('face_turn_speed').perform(context))
    scan_all = LaunchConfiguration('scan_all').perform(context).lower() == 'true'
    scoring = LaunchConfiguration('scoring').perform(context).lower() == 'true'
    far_robots = int(LaunchConfiguration('far_robots').perform(context))
    names = [f'tb3_{i}' for i in range(robots)]

    actions = [Node(package='search_and_rescue_comp_26', executable='marker_reporter', output='screen',
                    parameters=[{'robots': names}])]
    if scoring:
        actions.append(Node(package='scoring', executable='scoring', output='screen'))
    for name in names:
        actions.append(Node(package='search_and_rescue_comp_26', executable='navigator', namespace=name, output='screen',
                            parameters=[{'speed': speed, 'turn_speed': turn_speed, 'face_turn_speed': face_turn}]))
    if mission:
        actions.append(Node(package='search_and_rescue_comp_26', executable='mission', output='screen',
                            parameters=[{'robots': names, 'reserves': reserves, 'use_wall_colour': wall_colour,
                                         'wait_for_scoring': wait_scoring, 'scan_all': scan_all, 'far_robots': far_robots}]))
    if wall_colour:
        actions.append(Node(package='search_and_rescue_comp_26', executable='wall_colour', output='screen',
                            parameters=[{'robots': names}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='4'),
        DeclareLaunchArgument('reserves', default_value='0', description='How many of the robots wait as big fire helpers'),
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('scoring', default_value='true', description='Start the scoring node as well'),
        DeclareLaunchArgument('wait_for_scoring', default_value='true', description='Robots wait until the scoring node is running'),
        DeclareLaunchArgument('speed', default_value='0.26', description='Forward speed m/s (Turtlebot3 spec max 0.26)'),
        DeclareLaunchArgument('turn_speed', default_value='1.8', description='Max turn speed rad/s (Turtlebot3 spec max 1.82)'),
        DeclareLaunchArgument('face_turn_speed', default_value='1.0', description='Turn speed when looking around at a viewpoint (rad/s)'),
        DeclareLaunchArgument('scan_all', default_value='true', description='Look at all walls at a viewpoint, not only unseen ones'),
        DeclareLaunchArgument('use_wall_colour', default_value='false', description='Start the white wall detection and let the mission use it'),
        DeclareLaunchArgument('far_robots', default_value='2', description='How many robots (the last ones) start with the viewpoints furthest away'),
        OpaqueFunction(function=launch_setup),
    ])
