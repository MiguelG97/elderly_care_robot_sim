#!/usr/bin/env python3
"""
social_zones: turns each tracked person into a personal-space cost zone that
Nav2 can plan around.

Input:   /people/odom   nav_msgs/Odometry, one message per person
                        (from people_publisher now, a real detector in Stage B)
         /map           nav_msgs/OccupancyGrid (the saved house map; only its
                        size, resolution and origin are used)
Output:  /social_costmap  nav_msgs/OccupancyGrid with the same geometry as /map.
                        Each person adds an asymmetric Gaussian "personal space":
                        longer in front of them (more so the faster they walk),
                        shorter to the sides and behind.  Values stay BELOW 100
                        (default peak 95 at the body, 80 around it), so Nav2 reads
                        them as high but never forbidden cost: the robot prefers to
                        keep its distance but can still pass in a narrow corridor.
                        The person's real body is already marked as a solid
                        obstacle by the lidar; making the zone solid as well would
                        add Nav2's full safety margin around it and block the
                        corridor entirely.

Nav2 reads /social_costmap with an extra StaticLayer ("social_layer") using
use_maximum: true and trinary_costmap: false (see config/nav2_params_social.yaml).

Model: asymmetric Gaussian personal space, in the spirit of Kirby et al. (2009)
and later social navigation work.  All widths are parameters so they can be
varied in experiments.
"""
import array
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def social_grid(width, height, resolution, origin_x, origin_y, people,
                sigma_front=0.9, sigma_front_per_speed=0.6, sigma_side=0.45,
                sigma_rear=0.45, amplitude=80, body_radius=0.3, body_cost=95, cutoff=3.0,
                min_value=2):
    """Return an int8 grid (height x width) with personal-space costs 0..99.

    people: list of (x, y, heading, speed) in the grid's frame.
    """
    grid = np.zeros((height, width), dtype=np.float32)
    for (px, py, heading, speed) in people:
        i0 = max(int((px - cutoff - origin_x) / resolution), 0)
        i1 = min(int((px + cutoff - origin_x) / resolution) + 1, width)
        j0 = max(int((py - cutoff - origin_y) / resolution), 0)
        j1 = min(int((py + cutoff - origin_y) / resolution) + 1, height)
        if i0 >= i1 or j0 >= j1:
            continue
        xs = origin_x + (np.arange(i0, i1) + 0.5) * resolution
        ys = origin_y + (np.arange(j0, j1) + 0.5) * resolution
        gx, gy = np.meshgrid(xs - px, ys - py)
        c, s = math.cos(heading), math.sin(heading)
        fwd = c * gx + s * gy
        side = -s * gx + c * gy
        sf = sigma_front + sigma_front_per_speed * max(speed, 0.0)
        sx = np.where(fwd >= 0.0, sf, sigma_rear)
        val = amplitude * np.exp(-(fwd ** 2) / (2 * sx ** 2) - (side ** 2) / (2 * sigma_side ** 2))
        val[np.hypot(gx, gy) <= body_radius] = body_cost
        grid[j0:j1, i0:i1] = np.maximum(grid[j0:j1, i0:i1], val)
    grid[grid < min_value] = 0.0
    # Never 100: Nav2 would treat it as a wall and inflate it.
    return np.clip(np.rint(grid), 0, 99).astype(np.int8)


class SocialZones(Node):
    def __init__(self):
        super().__init__('social_zones')
        p = self.declare_parameters('', [
            ('people_topic', '/people/odom'),
            ('map_topic', '/map'),
            ('output_topic', '/social_costmap'),
            ('rate', 5.0),
            ('person_timeout', 1.0),
            ('sigma_front', 0.9),
            ('sigma_front_per_speed', 0.6),
            ('sigma_side', 0.45),
            ('sigma_rear', 0.45),
            ('amplitude', 80),
            ('body_radius', 0.3),
            ('body_cost', 95),
            ('cutoff', 3.0),
        ])
        self.params = {x.name: x.value for x in p}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_info = None
        self.map_frame = 'map'
        self.people = {}  # name -> (x, y, heading, speed, stamp_seconds)

        self.create_subscription(OccupancyGrid, self.params['map_topic'], self.on_map, latched)
        self.create_subscription(Odometry, self.params['people_topic'], self.on_person, 20)
        self.pub = self.create_publisher(OccupancyGrid, self.params['output_topic'], latched)
        self.create_timer(1.0 / self.params['rate'], self.tick)
        self.get_logger().info('Waiting for the map on ' + self.params['map_topic'])

    def on_map(self, msg):
        first = self.map_info is None
        self.map_info = msg.info
        self.map_frame = msg.header.frame_id or 'map'
        if first:
            i = msg.info
            self.get_logger().info(
                f'Map received: {i.width}x{i.height} cells at {i.resolution:.3f} m; '
                f'publishing {self.params["output_topic"]}')

    def on_person(self, msg):
        self.people[msg.child_frame_id or 'person'] = (
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            yaw_from_quat(msg.pose.pose.orientation), msg.twist.twist.linear.x,
            self.get_clock().now().nanoseconds * 1e-9)

    def tick(self):
        if self.map_info is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        timeout = self.params['person_timeout']
        active = [(x, y, h, v) for (x, y, h, v, t) in self.people.values() if now - t <= timeout]

        i = self.map_info
        P = self.params
        grid = social_grid(i.width, i.height, i.resolution,
                           i.origin.position.x, i.origin.position.y, active,
                           P['sigma_front'], P['sigma_front_per_speed'], P['sigma_side'],
                           P['sigma_rear'], P['amplitude'], P['body_radius'],
                           P['body_cost'], P['cutoff'])
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info = i
        msg.data = array.array('b', grid.ravel().tobytes())
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SocialZones()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()