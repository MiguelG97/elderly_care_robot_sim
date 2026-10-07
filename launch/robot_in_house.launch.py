"""
Start the AWS Small House and spawn bcr_bot in it.

  ros2 launch elderly_care_robot_sim robot_in_house.launch.py
  ros2 launch elderly_care_robot_sim robot_in_house.launch.py world:=small_house_no_people.world

Arguments:
  world      world file name inside the package's worlds/ folder
             (default small_house.world, which includes the walking resident)
  x, y, yaw  robot spawn pose in Gazebo world coordinates (default: east end of
             the corridor, facing west, toward the walking resident)
  camera     enable bcr_bot's depth camera (default False: saves a lot of CPU on
             a VM without a GPU; the 2D lidar is always on)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg = get_package_share_directory('elderly_care_robot_sim')
    bcr = get_package_share_directory('bcr_bot')
    gz = get_package_share_directory('ros_gz_sim')

    world = PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])

    house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items())

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bcr, 'launch', 'bcr_bot_gz_spawn.launch.py')),
        launch_arguments={
            'position_x': LaunchConfiguration('x'),
            'position_y': LaunchConfiguration('y'),
            'orientation_yaw': LaunchConfiguration('yaw'),
            'camera_enabled': LaunchConfiguration('camera'),
            'stereo_camera_enabled': 'False',
            'two_d_lidar_enabled': 'True',
            'odometry_source': 'world',
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='small_house.world'),
        DeclareLaunchArgument('x', default_value='2.94'),
        DeclareLaunchArgument('y', default_value='4.40'),
        DeclareLaunchArgument('yaw', default_value='3.14'),
        DeclareLaunchArgument('camera', default_value='False'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(bcr, 'models')),
        house,
        robot,
    ])