#!/usr/bin/env python3
"""
scan_check: is the laser data consistent with the home's map, and does wheel odometry keep
its heading? Uses the true pose, so AMCL plays no part.

Run while robot_in_house.launch.py is up (Nav2 not needed), ideally with the robot still:
  ros2 run elderly_care_robot_sim scan_check.py --ros-args -p use_sim_time:=true -p home:=guest_flat

It collects `scans` laser scans, places each one on the map at the robot's true pose, and
prints how far the laser points land from the map's walls. It then tries turning the scan
(yaw offset), stretching it (angle scale) and mirroring it, and reports the best fit:
  - best fit at offset 0, scale 1, not mirrored, points within a few cm: the laser is fine;
  - a better fit elsewhere: the scan's angles do not match where the rays really went.
Then, with spin_turns > 0, it turns the robot on the spot (through key_vel, so do not run
it while Nav2 is driving) and compares how far wheel odometry says it turned with how far it
really turned. Run it with the robot on its dock: there is room to turn there.
"""
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def read_map(folder):
    import yaml
    with open(os.path.join(folder, 'map.yaml')) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(folder, meta['image']), 'rb') as f:
        raw = f.read()
    header, pos = [], 0
    while len(header) < 4:
        end = raw.index(b'\n', pos)
        header += raw[pos:end].split()
        pos = end + 1
    w, h = int(header[1]), int(header[2])
    px = raw[pos:pos + w * h]
    occ = [px[(h - 1 - y) * w + x] == 0 for y in range(h) for x in range(w)]
    return w, h, meta['resolution'], meta['origin'][:2], occ


def distance_field(occ, w, h, res):
    """Metres from each cell to the nearest occupied cell (two-pass chamfer, 3-4 weights)."""
    big = 10 ** 9
    d = [0 if o else big for o in occ]
    for ys, xs, steps in ((range(h), range(w), ((-1, 0, 3), (0, -1, 3), (-1, -1, 4), (1, -1, 4))),
                          (range(h - 1, -1, -1), range(w - 1, -1, -1),
                           ((1, 0, 3), (0, 1, 3), (1, 1, 4), (-1, 1, 4)))):
        for y in ys:
            for x in xs:
                k = y * w + x
                for dx, dy, c in steps:
                    xx, yy = x + dx, y + dy
                    if 0 <= xx < w and 0 <= yy < h and d[yy * w + xx] + c < d[k]:
                        d[k] = d[yy * w + xx] + c
    return [v * res / 3 for v in d]


class ScanCheck(Node):
    def __init__(self):
        super().__init__('scan_check')
        p = self.declare_parameters('', [('home', 'guest_flat'), ('scans', 20), ('spin_turns', 2.0), ('spin_rate', 0.5)])
        self.p = {q.name: q.value for q in p}
        folder = os.path.join(get_package_share_directory('elderly_care_robot_sim'), 'homes', self.p['home'])
        self.w, self.h, self.res, (self.x0, self.y0), occ = read_map(folder)
        self.dist = distance_field(occ, self.w, self.h, self.res)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.truth = None
        self.truth_turned = self.odom_turned = 0.0
        self.last_truth_yaw = self.last_odom_yaw = None
        self.scans = []
        self.cmd = self.create_publisher(Twist, 'key_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 10)
        self.create_subscription(Odometry, '/mobile_base_controller/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_truth(self, msg):
        pose = msg.pose.pose
        yaw = yaw_of(pose.orientation)
        if self.last_truth_yaw is not None:
            self.truth_turned += wrap(yaw - self.last_truth_yaw)
        self.last_truth_yaw = yaw
        self.truth = (pose.position.x, pose.position.y, yaw)

    def on_odom(self, msg):
        yaw = yaw_of(msg.pose.pose.orientation)
        if self.last_odom_yaw is not None:
            self.odom_turned += wrap(yaw - self.last_odom_yaw)
        self.last_odom_yaw = yaw

    def on_scan(self, msg):
        if self.truth and len(self.scans) < self.p['scans']:
            self.scans.append((msg, self.truth))

    def mount(self, frame):
        t = self.tf.lookup_transform('base_footprint', frame, Time()).transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def fit(self, scan, truth, mount, offset=0.0, scale=1.0, mirror=False):
        """Distances (m) from each laser point to the nearest map wall."""
        rx, ry, ryaw = truth
        mx, my, myaw = mount
        lx = rx + mx * math.cos(ryaw) - my * math.sin(ryaw)
        ly = ry + mx * math.sin(ryaw) + my * math.cos(ryaw)
        lyaw = ryaw + myaw + offset
        n = len(scan.ranges)
        mid = scan.angle_min + scan.angle_increment * (n - 1) / 2
        out = []
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max) or math.isinf(r) or math.isnan(r):
                continue
            j = n - 1 - i if mirror else i
            a = mid + (scan.angle_min + j * scan.angle_increment - mid) * scale
            px, py = lx + r * math.cos(lyaw + a), ly + r * math.sin(lyaw + a)
            cx, cy = int((px - self.x0) / self.res), int((py - self.y0) / self.res)
            if 0 <= cx < self.w and 0 <= cy < self.h:
                out.append(self.dist[cy * self.w + cx])
            else:
                out.append(1.0)
        return out

    def run(self):
        self.get_logger().info(f"Collecting {self.p['scans']} scans...")
        while rclpy.ok() and len(self.scans) < self.p['scans']:
            rclpy.spin_once(self, timeout_sec=0.1)
        scan = self.scans[0][0]
        while rclpy.ok():
            try:
                mount = self.mount(scan.header.frame_id)
                break
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        n = len(scan.ranges)
        end = scan.angle_min + scan.angle_increment * (n - 1)
        print(f'\nLaser: frame {scan.header.frame_id}, {n} rays, angle_min {scan.angle_min:.4f}, '
              f'angle_max {scan.angle_max:.4f}, increment {scan.angle_increment:.6f}')
        print(f'  angle_min + increment x (rays - 1) = {end:.4f} '
              f'({"matches" if abs(end - scan.angle_max) < scan.angle_increment else "DOES NOT match"} angle_max)')
        print(f'  range {scan.range_min:.2f}-{scan.range_max:.2f} m; mounted at x {mount[0]:.3f} y {mount[1]:.3f} '
              f'yaw {mount[2]:.3f} from base_footprint')
        print(f'  robot (true pose) at x {self.scans[0][1][0]:.2f} y {self.scans[0][1][1]:.2f} '
              f'yaw {self.scans[0][1][2]:.2f}')

        def score(**kw):
            d = sorted(v for s, t in self.scans[::max(1, len(self.scans) // 5)] for v in self.fit(s, t, mount, **kw))
            # Capped so a few rays through a doorway do not decide the fit; a median was too
            # forgiving, since most rays land near some wall even when the scan is turned.
            return sum(min(v, 0.5) for v in d) / len(d), d[int(len(d) * 0.9)]

        med, p90 = score()
        print(f'\nAs published: points are {med * 100:.1f} cm from a wall on average, 90% within {p90 * 100:.1f} cm')
        best = (med, 0.0, 1.0, False)
        for mirror in (False, True):
            for scale in [1 + k * 0.02 for k in range(-10, 11)]:
                for offset in [k * 0.02 for k in range(-20, 21)]:
                    m = score(offset=offset, scale=scale, mirror=mirror)[0]
                    if m < best[0] - 1e-4:
                        best = (m, offset, scale, mirror)
        m, offset, scale, mirror = best
        print(f'Best fit: yaw offset {offset:+.2f} rad, angle scale {scale:.2f}, '
              f'{"mirrored" if mirror else "not mirrored"}: {m * 100:.1f} cm on average')
        if offset == 0 and scale == 1 and not mirror:
            print('  -> the laser matches the map at the true pose')
        else:
            print('  -> the scan fits better turned, stretched or mirrored: its angles are off')
        if self.p['spin_turns'] > 0:
            self.spin()

    def wait(self, seconds):
        end = self.get_clock().now().nanoseconds * 1e-9 + seconds
        while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin(self):
        target = 2 * math.pi * self.p['spin_turns']
        print(f"\nTurning {self.p['spin_turns']:g} times on the spot at {self.p['spin_rate']} rad/s...")
        self.truth_turned = self.odom_turned = 0.0
        twist = Twist()
        twist.angular.z = self.p['spin_rate']
        while rclpy.ok() and abs(self.truth_turned) < target:
            self.cmd.publish(twist)
            self.wait(0.05)
        self.cmd.publish(Twist())
        self.wait(2.0)
        diff = self.odom_turned - self.truth_turned
        print(f'True heading turned {self.truth_turned:+.3f} rad, wheel odometry says {self.odom_turned:+.3f} rad: '
              f'odometry off by {diff:+.3f} rad ({math.degrees(diff):+.1f} deg) over {self.p["spin_turns"]:g} turns')


def main():
    rclpy.init()
    node = ScanCheck()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
