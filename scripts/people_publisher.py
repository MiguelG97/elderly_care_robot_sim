#!/usr/bin/env python3
"""
people_publisher: ground-truth people tracking.

Reads each actor's scripted trajectory straight from the world SDF file and,
using simulation time, publishes where every person is *right now* and how
fast they are moving.

Outputs:
  /people/odom     nav_msgs/Odometry, one message per person per tick
                   header.frame_id = map frame, child_frame_id = person name,
                   pose = position + heading, twist.linear.x = walking speed
                   (twist is in the person's own frame, as Odometry expects)
  /people/markers  visualization_msgs/MarkerArray for RViz

Parameters:
  world_file      path to the .world/.sdf containing the actors
  actor_names     list of actor names to track (empty = all actors in the file)
  map_frame       frame to publish in (default "map")
  robot_spawn_x/y/yaw  where the robot was spawned in Gazebo world coordinates
                  when the map was built; used to convert Gazebo world
                  coordinates into map coordinates
  rate            publish rate in Hz (default 20)
Run with use_sim_time:=true so the clock matches the actors' script time.
"""
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ActorScript:
    """A scripted actor trajectory: a list of (t, x, y, yaw) in world coords."""

    def __init__(self, name, waypoints, loop, delay):
        self.name = name
        self.wp = waypoints
        self.loop = loop
        self.delay = delay
        self.period = waypoints[-1][0] if waypoints else 0.0

    def state(self, t):
        """Return (x, y, heading, vx, vy) at script time t (world frame)."""
        t -= self.delay
        if t < 0 or self.period <= 0:
            x, y, yaw = self.wp[0][1:]
            return x, y, yaw, 0.0, 0.0
        if self.loop:
            t = math.fmod(t, self.period)
        elif t >= self.period:
            x, y, yaw = self.wp[-1][1:]
            return x, y, yaw, 0.0, 0.0
        for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(self.wp, self.wp[1:]):
            if t0 <= t <= t1:
                dt = t1 - t0
                if dt <= 1e-9:
                    return x1, y1, a1, 0.0, 0.0
                s = (t - t0) / dt
                vx, vy = (x1 - x0) / dt, (y1 - y0) / dt
                yaw = a0 + s * wrap(a1 - a0)
                return x0 + s * (x1 - x0), y0 + s * (y1 - y0), yaw, vx, vy
        x, y, yaw = self.wp[-1][1:]
        return x, y, yaw, 0.0, 0.0


def load_actors(world_file, wanted):
    """Parse <actor> trajectories from an SDF world file."""
    root = ET.parse(world_file).getroot()
    actors = []
    for actor in root.iter('actor'):
        name = actor.get('name')
        if wanted and name not in wanted:
            continue
        script = actor.find('script')
        if script is None:
            continue
        loop = (script.findtext('loop', 'true').strip().lower() == 'true')
        delay = float(script.findtext('delay_start', '0') or 0)
        waypoints, offset = [], 0.0
        # Multiple <trajectory> blocks play one after another.
        trajs = sorted(script.findall('trajectory'),
                       key=lambda tr: int(tr.get('id', '0')))
        for tr in trajs:
            last = 0.0
            for wp in tr.findall('waypoint'):
                t = float(wp.findtext('time'))
                p = [float(v) for v in wp.findtext('pose').split()]
                waypoints.append((offset + t, p[0], p[1], p[5]))
                last = t
            offset += last
        if len(waypoints) >= 2:
            actors.append(ActorScript(name, waypoints, loop, delay))
    return actors


class PeoplePublisher(Node):
    def __init__(self):
        super().__init__('people_publisher')
        self.declare_parameter('world_file', '')
        self.declare_parameter('actor_names', [''])
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_spawn_x', 0.0)
        self.declare_parameter('robot_spawn_y', 0.0)
        self.declare_parameter('robot_spawn_yaw', 0.0)
        self.declare_parameter('rate', 20.0)

        world = self.get_parameter('world_file').value
        if not world or not os.path.isfile(world):
            raise RuntimeError(f'world_file not found: "{world}"')
        wanted = [n for n in self.get_parameter('actor_names').value if n]
        self.actors = load_actors(world, wanted)
        if not self.actors:
            raise RuntimeError(f'No scripted actors found in {world}')

        self.frame = self.get_parameter('map_frame').value
        self.sx = self.get_parameter('robot_spawn_x').value
        self.sy = self.get_parameter('robot_spawn_y').value
        self.syaw = self.get_parameter('robot_spawn_yaw').value

        self.odom_pub = self.create_publisher(Odometry, '/people/odom', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/people/markers', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.get_logger().info(
            'Tracking ' + ', '.join(a.name for a in self.actors) + f' from {world}')

    def to_map(self, x, y, yaw, vx, vy):
        """Gazebo world coords -> map coords (map origin = robot spawn pose)."""
        c, s = math.cos(-self.syaw), math.sin(-self.syaw)
        dx, dy = x - self.sx, y - self.sy
        return (c * dx - s * dy, s * dx + c * dy, wrap(yaw - self.syaw),
                c * vx - s * vy, s * vx + c * vy)

    def tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        stamp = now.to_msg()
        markers = MarkerArray()
        for i, actor in enumerate(self.actors):
            x, y, yaw, vx, vy = self.to_map(*actor.state(t))
            speed = math.hypot(vx, vy)
            heading = math.atan2(vy, vx) if speed > 0.05 else yaw
            q = yaw_to_quat(heading)

            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = self.frame
            od.child_frame_id = actor.name
            od.pose.pose.position.x, od.pose.pose.position.y = x, y
            (od.pose.pose.orientation.x, od.pose.pose.orientation.y,
             od.pose.pose.orientation.z, od.pose.pose.orientation.w) = q
            od.twist.twist.linear.x = speed  # forward speed in the person's frame
            self.odom_pub.publish(od)

            markers.markers += self.make_markers(i, actor.name, stamp, x, y, q, speed)
        self.marker_pub.publish(markers)

    def make_markers(self, i, name, stamp, x, y, q, speed):
        life = Duration(sec=0, nanosec=300_000_000)
        body = Marker()
        body.header.frame_id, body.header.stamp = self.frame, stamp
        body.ns, body.id, body.type = 'people', i * 3, Marker.CYLINDER
        body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.85
        body.pose.orientation.w = 1.0
        body.scale.x = body.scale.y = 0.5
        body.scale.z = 1.7
        body.color.r, body.color.g, body.color.b, body.color.a = 0.1, 0.8, 0.3, 0.6
        body.lifetime = life

        arrow = Marker()
        arrow.header.frame_id, arrow.header.stamp = self.frame, stamp
        arrow.ns, arrow.id, arrow.type = 'people', i * 3 + 1, Marker.ARROW
        arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = x, y, 0.1
        (arrow.pose.orientation.x, arrow.pose.orientation.y,
         arrow.pose.orientation.z, arrow.pose.orientation.w) = q
        arrow.scale.x = max(speed, 0.05)  # arrow length = distance covered in 1 s
        arrow.scale.y = arrow.scale.z = 0.08
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.5, 0.0, 0.9
        arrow.lifetime = life

        label = Marker()
        label.header.frame_id, label.header.stamp = self.frame, stamp
        label.ns, label.id, label.type = 'people', i * 3 + 2, Marker.TEXT_VIEW_FACING
        label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 1.95
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f'{name} {speed:.2f} m/s'
        label.lifetime = life
        return [body, arrow, label]


def main():
    rclpy.init()
    node = None
    try:
        node = PeoplePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()