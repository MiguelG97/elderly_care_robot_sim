"""
Start the AWS Small House and spawn PAL Robotics' TIAGo (PMB2 base, no arm) in it.

  ros2 launch elderly_care_robot_sim robot_in_house.launch.py
  ros2 launch elderly_care_robot_sim robot_in_house.launch.py headless:=True

Then navigate with:
  ros2 launch elderly_care_robot_sim navigation.launch.py social:=true
or drive by hand (key_vel has priority over Nav2 in TIAGo's twist_mux):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=key_vel

Arguments:
  world      world file name inside the package's worlds/ folder (default small_house.world)
  x, y, yaw  spawn pose in Gazebo world coordinates (default: east end of the
             corridor, facing west, toward the walking resident)
  camera     enable the head RGB-D camera (default False: saves a lot of CPU on a
             VM without a GPU; the base laser is always on)
  headless   run the Gazebo server without its GUI (default False)

The robot model, controllers and bridge come from the Tiago-Harmonic packages
(https://github.com/Tiago-Harmonic, jazzy, Apache-2.0); see tiago.repos.
Their own tiago_gazebo.launch.py always spawns at the world origin and starts its
own world, so this file spawns TIAGo itself and reuses only their bringup.

Topics: /scan_raw (base laser, SICK TIM571), /mobile_base_controller/odom (wheel
odometry, also odom -> base_footprint TF), /ground_truth/odom (true pose in the
map frame, see scripts/gz_description.py), /mobile_base_controller/cmd_vel
(TwistStamped; fed from twist_mux through twist_stamper).
"""
import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def is_true(arg):
    return ["'", LaunchConfiguration(arg), "'.lower() in ('true', '1')"]


def generate_launch_description():
    pkg = get_package_share_directory('elderly_care_robot_sim')
    gz = get_package_share_directory('ros_gz_sim')
    tiago_bringup = get_package_share_directory('tiago_bringup')

    world = PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])
    server_only = PythonExpression(["'-s ' if "] + is_true('headless') + [" else ''"])
    camera_model = PythonExpression(["'orbbec-astra' if "] + is_true('camera') + [" else 'no-camera'"])

    house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', server_only, world]}.items())

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tiago_bringup, 'launch', 'tiago_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'is_public_sim': 'True',
            'base_type': 'pmb2',
            'arm_type': 'no-arm',
            'end_effector': 'no-end-effector',
            'ft_sensor': 'no-ft-sensor',
            'laser_model': 'sick-571',
            'camera_model': camera_model,
        }.items())

    gz_description = Node(
        package='elderly_care_robot_sim', executable='gz_description.py', output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'tiago', '-topic', 'robot_description_gz',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.0', '-Y', LaunchConfiguration('yaw')],
        parameters=[{'use_sim_time': True}])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge_ros_gz',
        output='screen',
        parameters=[{'config_file': os.path.join(tiago_bringup, 'config', 'bridge', 'tiago_bridge.yaml'),
                     'use_sim_time': True}])

    ground_truth_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge_ground_truth',
        output='screen',
        arguments=['/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        parameters=[{'use_sim_time': True}])

    camera_bridges = [
        Node(package='ros_gz_image', executable='image_bridge', name=name, output='screen',
             arguments=[topic], parameters=[{'use_sim_time': True}],
             condition=IfCondition(PythonExpression(is_true('camera'))))
        for name, topic in [('bridge_gz_ros_camera_image', '/head_front_camera/image'),
                            ('bridge_gz_ros_camera_depth', '/head_front_camera/depth_image')]]

    # On Jazzy the base controller only accepts TwistStamped.
    stamper = Node(
        package='twist_stamper', executable='twist_stamper', output='screen',
        parameters=[{'frame_id': 'base_footprint', 'use_sim_time': True}],
        remappings=[('cmd_vel_in', '/cmd_vel_muxed'),
                    ('cmd_vel_out', '/mobile_base_controller/cmd_vel')])

    # Gazebo resolves the meshes' package:// URIs through these share folders.
    mesh_paths = [os.path.join(get_package_prefix(p), 'share')
                  for p in ('tiago_description', 'pmb2_description')]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='small_house.world'),
        DeclareLaunchArgument('x', default_value='2.94'),
        DeclareLaunchArgument('y', default_value='4.40'),
        DeclareLaunchArgument('yaw', default_value='3.14'),
        DeclareLaunchArgument('camera', default_value='False'),
        DeclareLaunchArgument('headless', default_value='False'),
        SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH',
                               os.path.join(pkg, 'config', 'gz_server.config')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
        *[AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', p) for p in mesh_paths],
        house,
        bringup,
        gz_description,
        spawn,
        bridge,
        ground_truth_bridge,
        *camera_bridges,
        stamper,
    ])
