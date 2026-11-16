#!/usr/bin/env python3
"""
gz_description: TIAGo's robot_description plus a Gazebo ground-truth odometry plugin.

Input:   /robot_description      std_msgs/String (from robot_state_publisher)
Output:  /robot_description_gz   std_msgs/String, the same URDF with Gazebo's
                                 OdometryPublisher added; robot_in_house.launch.py
                                 spawns the robot from this topic.

The plugin publishes the robot's true pose and velocity on the Gazebo topic
/ground_truth/odom, bridged to ROS as nav_msgs/Odometry, for analyze_runs.py.
TIAGo's own odometry comes from its wheels and drifts, so it cannot be used to
measure robot-person distances. OdometryPublisher reports the Gazebo world pose,
and the saved map is in world coordinates, so the frame is 'map'.

Gazebo's entity/system/add service could attach the plugin after spawning, but
it needs the model's numeric entity id, which a launch file cannot easily get.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

PLUGIN = """
  <gazebo>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>map</odom_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <odom_topic>/ground_truth/odom</odom_topic>
      <tf_topic>/ground_truth/tf</tf_topic>
      <odom_publish_frequency>20</odom_publish_frequency>
      <dimensions>2</dimensions>
    </plugin>
  </gazebo>
"""


class GzDescription(Node):
    def __init__(self):
        super().__init__('gz_description')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, '/robot_description_gz', latched)
        self.create_subscription(String, '/robot_description', self.on_description, latched)

    def on_description(self, msg):
        urdf = msg.data
        end = urdf.rfind('</robot>')
        if end < 0:
            self.get_logger().error('robot_description has no </robot> tag')
            return
        self.pub.publish(String(data=urdf[:end] + PLUGIN + urdf[end:]))
        self.get_logger().info('Published /robot_description_gz with ground-truth odometry')


def main():
    rclpy.init()
    rclpy.spin(GzDescription())


if __name__ == '__main__':
    main()
