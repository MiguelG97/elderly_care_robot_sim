"""
Nav2 in the small house, with people tracking and personal-space zones.

Start the simulation first (with the walking resident):
  ros2 launch elderly_care_robot_sim robot_in_house.launch.py
Then, in a second terminal, one of:
  ros2 launch elderly_care_robot_sim navigation.launch.py social:=false   # BASELINE
  ros2 launch elderly_care_robot_sim navigation.launch.py social:=true    # HUMAN-AWARE

Both modes run people_publisher and social_zones, so RViz shows the same people
markers and zones; only social:=true feeds the zones into Nav2's costmaps.

Localisation: the map is in Gazebo world coordinates. TIAGo's odometry comes from
its wheels and starts at its spawn pose, so AMCL localises it on the map, starting
from the spawn pose given by x, y, yaw (same defaults as robot_in_house.launch.py).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    pkg = get_package_share_directory('elderly_care_robot_sim')
    nav2 = get_package_share_directory('nav2_bringup')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')

    social = arg('social').lower() in ('true', '1')
    params = os.path.join(pkg, 'config', 'nav2_params_social.yaml' if social else 'nav2_params.yaml')

    sim_time = {'use_sim_time': True}

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[sim_time, {'yaml_filename': map_yaml}])

    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[params, sim_time, {'initial_pose': {
            'x': float(arg('x')), 'y': float(arg('y')), 'z': 0.0, 'yaw': float(arg('yaw'))}}])

    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[sim_time, {'autostart': True,
                               'node_names': ['map_server', 'amcl']}])

    # Only the Nav2 servers the demo needs (planner, controller, recovery behaviours,
    # behaviour tree). Nav2's full navigation_launch.py also starts the smoother,
    # velocity smoother, collision monitor, route, docking and waypoint servers;
    # on a 2-vCPU VM those extra processes starve the CPU and the lifecycle
    # manager shuts Nav2 down when one misses its heartbeat.
    # Through TIAGo's twist_mux, so teleop on key_vel can always take over.
    to_robot = [('cmd_vel', '/nav_vel')]
    nav_nodes = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
    navigation = [
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[params, sim_time], remappings=to_robot),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params, sim_time]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[params, sim_time], remappings=to_robot),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[params, sim_time]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[sim_time, {'autostart': True, 'node_names': nav_nodes,
                                    # 0 = do not shut Nav2 down when a server is briefly
                                    # too busy to answer its heartbeat (slow VM)
                                    'bond_timeout': 0.0}]),
    ]

    people = Node(
        package='elderly_care_robot_sim', executable='people_publisher.py', name='people_publisher',
        output='screen',
        parameters=[sim_time, {
            'world_file': os.path.join(pkg, 'worlds', 'small_house.world'),
            'actor_names': ['resident'],
            'map_frame': 'map'}])

    zones = Node(
        package='elderly_care_robot_sim', executable='social_zones.py', name='social_zones',
        output='screen', parameters=[sim_time])

    saved_view = os.path.join(pkg, 'config', 'nav_view.rviz')
    view = saved_view if os.path.isfile(saved_view) else os.path.join(nav2, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', view],
        parameters=[sim_time], condition=IfCondition(rviz))

    return [map_server, amcl, map_lifecycle, *navigation, people, zones, rviz_node]


def generate_launch_description():
    pkg = get_package_share_directory('elderly_care_robot_sim')
    return LaunchDescription([
        DeclareLaunchArgument('social', default_value='false',
                              description='true = human-aware (zones in costmaps), false = baseline'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'small_house.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        # AMCL's initial pose: the spawn pose in Gazebo world coordinates
        DeclareLaunchArgument('x', default_value='2.94'),
        DeclareLaunchArgument('y', default_value='4.40'),
        DeclareLaunchArgument('yaw', default_value='3.14'),
        OpaqueFunction(function=launch_setup),
    ])