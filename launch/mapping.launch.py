"""
Map the house: house (without people) + bcr_bot + SLAM Toolbox + RViz.

  ros2 launch elderly_care_robot_sim mapping.launch.py

Then drive with teleop in another terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/bcr_bot/cmd_vel
and save the map when it looks complete:
  ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/elderly_care_robot_sim/maps/small_house \
    --ros-args -p use_sim_time:=true

Uses small_house_no_people.world so the walking resident does not get drawn
into the map as a wall.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('elderly_care_robot_sim')
    bcr = get_package_share_directory('bcr_bot')
    slam = get_package_share_directory('slam_toolbox')

    robot_in_house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'robot_in_house.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items())

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(pkg, 'config', 'mapper_params.yaml'),
        }.items())

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(bcr, 'rviz', 'map.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='small_house_no_people.world'),
        DeclareLaunchArgument('x', default_value='2.94'),
        DeclareLaunchArgument('y', default_value='4.40'),
        DeclareLaunchArgument('yaw', default_value='3.14'),
        robot_in_house,
        slam_toolbox,
        rviz,
    ])