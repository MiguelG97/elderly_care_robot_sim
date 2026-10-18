"""
Nav2 for bcr_bot in the small house, with people tracking and personal-space zones.

Start the simulation first (with the walking resident):
  ros2 launch elderly_care_robot_sim robot_in_house.launch.py
Then, in a second terminal, one of:
  ros2 launch elderly_care_robot_sim navigation.launch.py social:=false   # BASELINE
  ros2 launch elderly_care_robot_sim navigation.launch.py social:=true    # HUMAN-AWARE

Both modes run people_publisher and social_zones, so RViz shows the same people
markers and zones; only social:=true feeds the zones into Nav2's costmaps.

Localisation: bcr_bot's odometry is ground truth from Gazebo (odom = Gazebo world
frame) and the map was built starting from that frame, so map -> odom is the
identity transform; no AMCL needed (same approach as bcr_bot's own nav2.launch.py).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('elderly_care_robot_sim')
    nav2 = get_package_share_directory('nav2_bringup')

    social = LaunchConfiguration('social')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')

    params = PythonExpression([
        "'", os.path.join(pkg, 'config', 'nav2_params_social.yaml'), "' if '", social,
        "'.lower() in ('true', '1') else '", os.path.join(pkg, 'config', 'nav2_params.yaml'), "'"])

    sim_time = {'use_sim_time': True}

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[sim_time, {'yaml_filename': map_yaml}])

    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[sim_time, {'autostart': True, 'node_names': ['map_server']}])

    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[sim_time])

    # Only the Nav2 servers the demo needs (planner, controller, recovery behaviours,
    # behaviour tree). Nav2's full navigation_launch.py also starts the smoother,
    # velocity smoother, collision monitor, route, docking and waypoint servers;
    # on a 2-vCPU VM those extra processes starve the CPU and the lifecycle
    # manager shuts Nav2 down when one misses its heartbeat.
    to_robot = [('cmd_vel', '/bcr_bot/cmd_vel')]
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

    # Use our saved RViz layout (people markers + social zones) if it exists,
    # otherwise Nav2's default view.
    saved_view = os.path.join(pkg, 'config', 'nav_view.rviz')
    view = saved_view if os.path.isfile(saved_view) else os.path.join(nav2, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', view],
        parameters=[sim_time], condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument('social', default_value='false',
                              description='true = human-aware (zones in costmaps), false = baseline'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'small_house.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        map_server, map_lifecycle, map_to_odom, *navigation, people, zones, rviz_node,
    ])