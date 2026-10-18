#!/usr/bin/env python3
"""
send_goal: send the same navigation goal at the same moment of the person's
walking loop, so baseline and human-aware runs start under identical conditions.

  ros2 run elderly_care_robot_sim send_goal.py --ros-args -p use_sim_time:=true
  ros2 run elderly_care_robot_sim send_goal.py --ros-args -p use_sim_time:=true -p x:=-0.9 -p y:=4.4

Parameters:
  x, y, yaw      goal pose in the map frame (default: west end of the corridor,
                 facing west; the robot starts at the east end)
  loop_period    length of the resident's walking loop in seconds (11.57 for the
                 current actor script)
  phase          point in the loop at which to send the goal (0.0 = the person is
                 at the west end and starts walking east, toward the robot)
  min_wait       wait at least this many sim seconds after startup before sending,
                 so Nav2 has settled
Prints the result and how long the robot took (sim time).
"""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class SendGoal(Node):
    def __init__(self):
        super().__init__('send_goal')
        p = self.declare_parameters('', [
            ('x', -0.9), ('y', 4.40), ('yaw', math.pi),
            ('loop_period', 11.57), ('phase', 0.0), ('min_wait', 5.0)])
        self.p = {q.name: q.value for q in p}
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_for_phase(self):
        period, phase = self.p['loop_period'], self.p['phase']
        while rclpy.ok() and self.sim_now() <= 0.0:          # wait for /clock
            rclpy.spin_once(self, timeout_sec=0.1)
        earliest = self.sim_now() + self.p['min_wait']
        k = math.ceil((earliest - phase) / period)
        target = k * period + phase
        self.get_logger().info(f'Sending goal at sim time {target:.2f} s (loop phase {phase:.2f} s)')
        while rclpy.ok() and self.sim_now() < target:
            rclpy.spin_once(self, timeout_sec=0.02)

    def run(self):
        self.get_logger().info('Waiting for Nav2 (navigate_to_pose)...')
        self.client.wait_for_server()
        self.wait_for_phase()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.p['x']
        goal.pose.pose.position.y = self.p['y']
        goal.pose.pose.orientation.z = math.sin(self.p['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(self.p['yaw'] / 2)

        start = self.sim_now()
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return
        self.get_logger().info(f'Goal accepted: ({self.p["x"]:.2f}, {self.p["y"]:.2f})')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(result.result().status,
                                                                   str(result.result().status))
        self.get_logger().info(f'Result: {status} after {self.sim_now() - start:.1f} s (sim time)')


def main():
    rclpy.init()
    node = SendGoal()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()