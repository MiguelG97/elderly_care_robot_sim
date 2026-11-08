#!/usr/bin/env python3
"""
task_runner: give TIAGo household jobs in a scanned home and call it back to its dock at
random moments, logging every job and every return.

Start the home with the robot on its dock (the converter prints the dock pose) and Nav2:
  ros2 launch elderly_care_robot_sim robot_in_house.launch.py world:=../homes/guest_flat/guest_flat.world \
    x:=-2.35 y:=0.59 yaw:=0.00
  ros2 launch elderly_care_robot_sim navigation.launch.py world:=../homes/guest_flat/guest_flat.world \
    map:=$(ros2 pkg prefix elderly_care_robot_sim)/share/elderly_care_robot_sim/homes/guest_flat/map.yaml \
    x:=-2.35 y:=0.59 yaw:=0.00
Then one of:
  ros2 run elderly_care_robot_sim task_runner.py --ros-args -p use_sim_time:=true -p home:=guest_flat \
    -p task:=patrol -p returns:=5 -p seed:=1
  ros2 run elderly_care_robot_sim task_runner.py --ros-args -p use_sim_time:=true -p home:=guest_flat \
    -p task:=room -p room:=Bathroom
  ros2 run elderly_care_robot_sim task_runner.py --ros-args -p use_sim_time:=true -p home:=guest_flat \
    -p task:=resident

Tasks (goals come from the home's annotations.yaml, written by tools/scan_to_world.py):
  patrol    visit the patrol goals (every room the robot can reach), nearest first, looping
  room      go to the most open spot of the named room
  resident  stand approach_distance from the resident, facing them, as if handing something
            over; repeated every resident_wait seconds while returns are still due
  mixed     a random job each time: a room's patrol goal or the resident

Return to dock: with returns > 0, a "go back to the dock" call comes at random sim times,
gap_min to gap_max seconds apart (drawn from seed). The robot drops its job, gets
dock_timeout seconds to reach the dock, stays rest seconds, then carries on. The run ends
after that many returns, or after duration sim seconds if set (0 = no limit). With
returns = 0, room and resident do their job once and exit.

Docked = Nav2 succeeded AND the true pose (/ground_truth/odom, not AMCL) is within
dock_xy_tol and dock_yaw_tol of the dock pose. The defaults equal Nav2's goal checker,
which cannot stop more precisely; the final errors are logged for stricter thresholds.

Output in log_dir/<run_id>/ (run_id defaults to <home>_<task>_seed<seed>):
  events.csv  one row per job or return: times, result, failure reason, Nav2 error, path
              driven (ground truth), final error, and where the robot and resident were
              (room, nearest doorway) when it started
  run.yaml    every parameter, ROS distro, start time and the checksums of the home's
              files: what is needed to repeat the run
Each row is also published as JSON on /task_events, so a bag holds it. Record with:
  ros2 bag record -o <dir> /ground_truth/odom /people/odom /task_events /clock
"""
import csv
import hashlib
import json
import math
import os
import random
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

FIELDS = ['run_id', 'home', 'social', 'seed', 'n', 'kind', 'target', 'goal_x', 'goal_y', 'goal_yaw',
          't_start', 't_end', 'duration_s', 'result', 'reason', 'nav2_status', 'nav2_error',
          'path_m', 'end_x', 'end_y', 'err_xy_m', 'err_yaw_rad',
          'robot_x', 'robot_y', 'robot_room', 'resident_x', 'resident_y', 'resident_room',
          'resident_doorway', 'resident_dist_m']
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
          GoalStatus.STATUS_ABORTED: 'aborted'}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def read_pgm(path):
    """Returns (width, height, cells) with cell k = y * width + x and y up, like the converter."""
    with open(path, 'rb') as f:
        raw = f.read()
    header, pos = [], 0
    while len(header) < 4:
        end = raw.index(b'\n', pos)
        header += raw[pos:end].split()
        pos = end + 1
    w, h = int(header[1]), int(header[2])
    px = raw[pos:pos + w * h]
    return w, h, bytes(b for y in range(h - 1, -1, -1) for b in px[y * w:(y + 1) * w])


class Home:
    def __init__(self, folder):
        self.folder = folder
        with open(os.path.join(folder, 'annotations.yaml')) as f:
            self.ann = yaml.safe_load(f)
        g = self.ann['grid']
        self.res, (self.x0, self.y0) = g['resolution'], g['origin']
        self.nx, self.ny, self.map = read_pgm(os.path.join(folder, 'map.pgm'))
        _, _, self.rooms = read_pgm(os.path.join(folder, 'rooms.pgm'))
        self.names = {r['id']: r['name'] for r in self.ann['rooms']}
        self.dock = self.ann.get('dock')
        self.patrol = self.ann.get('patrol') or []

    def cell(self, x, y):
        cx, cy = int((x - self.x0) / self.res), int((y - self.y0) / self.res)
        return cy * self.nx + cx if 0 <= cx < self.nx and 0 <= cy < self.ny else None

    def room(self, x, y):
        k = self.cell(x, y)
        return self.names.get(self.rooms[k], '') if k is not None else 'outside'

    def doorway(self, x, y, within=0.6):
        """The doorway a person at (x, y) stands in or next to, as 'A / B', else ''."""
        near = [(math.hypot(d['x'] - x, d['y'] - y), d) for d in self.ann.get('doors') or []]
        near = [n for n in near if n[0] <= within]
        return ' / '.join(min(near, key=lambda n: n[0])[1]['between']) if near else ''

    def fits(self, x, y, radius):
        """True if every map cell under a disc of this radius is free."""
        r = int(math.ceil(radius / self.res))
        c = self.cell(x, y)
        if c is None:
            return False
        cx, cy = c % self.nx, c // self.nx
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if (dx * dx + dy * dy) * self.res ** 2 > radius ** 2:
                    continue
                xx, yy = cx + dx, cy + dy
                if not (0 <= xx < self.nx and 0 <= yy < self.ny) or self.map[yy * self.nx + xx] != 254:
                    return False
        return True

    def checksums(self):
        out = {}
        for name in sorted(os.listdir(self.folder)):
            path = os.path.join(self.folder, name)
            if os.path.isfile(path) and not name.endswith('.png'):
                with open(path, 'rb') as f:
                    out[name] = hashlib.sha256(f.read()).hexdigest()[:16]
        return out


class TaskRunner(Node):
    def __init__(self):
        super().__init__('task_runner')
        p = self.declare_parameters('', [
            ('home', 'guest_flat'), ('task', 'patrol'), ('room', ''), ('returns', 0), ('seed', 0),
            ('gap_min', 60.0), ('gap_max', 240.0), ('dock_timeout', 180.0), ('task_timeout', 180.0),
            ('rest', 10.0), ('duration', 0.0), ('resident_wait', 20.0),
            ('approach_distance', 1.0), ('robot_radius', 0.275),
            ('dock_xy_tol', 0.25), ('dock_yaw_tol', 0.25),
            ('social', 'unknown'), ('log_dir', os.path.expanduser('~/runs')), ('run_id', '')])
        self.p = {q.name: q.value for q in p}
        if self.p['task'] not in ('patrol', 'room', 'resident', 'mixed'):
            raise SystemExit(f"task must be patrol, room, resident or mixed, not {self.p['task']!r}")
        share = get_package_share_directory('elderly_care_robot_sim')
        self.home = Home(os.path.join(share, 'homes', self.p['home']))
        if not self.home.dock and self.p['returns'] > 0:
            raise SystemExit(f"{self.p['home']}: annotations.yaml has no dock pose; set one by hand")
        if not self.home.patrol and self.p['task'] in ('patrol', 'mixed', 'room'):
            raise SystemExit(f"{self.p['home']}: annotations.yaml has no patrol goals; rerun scan_to_world.py")
        if self.p['task'] == 'room' and self.p['room'] not in {g['room'] for g in self.home.patrol}:
            rooms = sorted({g['room'] for g in self.home.patrol})
            raise SystemExit(f"room must be one the robot can reach: {', '.join(rooms)}")
        self.rng = random.Random(self.p['seed'])
        self.run_id = self.p['run_id'] or f"{self.p['home']}_{self.p['task']}_seed{self.p['seed']}"

        self.robot = None          # (x, y, yaw) from ground truth
        self.odometer = 0.0        # metres driven, ground truth
        self.resident = None       # (x, y)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_robot, 10)
        self.create_subscription(Odometry, '/people/odom', self.on_person, 10)
        self.events = self.create_publisher(String, '/task_events', 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.n = 0
        self.next_call = None

    def on_robot(self, msg):
        q, pos = msg.pose.pose.orientation, msg.pose.pose.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.robot:
            self.odometer += math.hypot(pos.x - self.robot[0], pos.y - self.robot[1])
        self.robot = (pos.x, pos.y, yaw)

    def on_person(self, msg):
        if msg.child_frame_id in ('resident', ''):
            self.resident = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, seconds):
        end = self.now() + seconds
        while rclpy.ok() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call_due(self):
        return self.next_call is not None and self.now() >= self.next_call

    def schedule_call(self):
        if self.n_returns < self.p['returns']:
            self.next_call = self.now() + self.rng.uniform(self.p['gap_min'], self.p['gap_max'])
        else:
            self.next_call = None

    # Navigation --------------------------------------------------------------------------

    def send(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        return handle if handle and handle.accepted else None

    def cancel(self, handle):
        future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def navigate(self, x, y, yaw, timeout, interruptible=True, retarget=None):
        """Drive to (x, y, yaw). Returns (outcome, nav2_status, nav2_error), outcome one of
        succeeded, aborted, canceled, rejected, timeout, interrupted (a dock call came).
        retarget() may return a new (x, y, yaw) while driving; the goal is then replaced."""
        handle = self.send(x, y, yaw)
        if handle is None:
            return 'rejected', '', ''
        result = handle.get_result_async()
        start, last_check = self.now(), self.now()
        while rclpy.ok() and not result.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.now() - start > timeout:
                self.cancel(handle)
                return 'timeout', '', ''
            if interruptible and self.call_due():
                self.cancel(handle)
                return 'interrupted', '', ''
            if retarget and self.now() - last_check >= 1.0:
                last_check = self.now()
                new = retarget()
                if new:
                    newer = self.send(*new)
                    if newer is not None:
                        handle, result = newer, newer.get_result_async()
        r = result.result()
        status = STATUS.get(r.status, str(r.status))
        code = getattr(r.result, 'error_code', 0)
        msg = getattr(r.result, 'error_msg', '')
        return status, status, f'{code} {msg}'.strip() if code else ''

    # Logging -----------------------------------------------------------------------------

    def where(self):
        """Where the robot and resident are now, for the log."""
        out = {}
        if self.robot:
            out.update(robot_x=round(self.robot[0], 3), robot_y=round(self.robot[1], 3),
                       robot_room=self.home.room(*self.robot[:2]))
        if self.resident:
            rx, ry = self.resident
            out.update(resident_x=round(rx, 3), resident_y=round(ry, 3), resident_room=self.home.room(rx, ry),
                       resident_doorway=self.home.doorway(rx, ry))
            if self.robot:
                out['resident_dist_m'] = round(math.hypot(rx - self.robot[0], ry - self.robot[1]), 3)
        return out

    def log(self, row):
        self.n += 1
        row.update(run_id=self.run_id, home=self.p['home'], social=self.p['social'], seed=self.p['seed'], n=self.n)
        with open(self.csv_path, 'a', newline='') as f:
            csv.DictWriter(f, FIELDS, extrasaction='ignore').writerow(row)
        self.events.publish(String(data=json.dumps(row)))
        self.get_logger().info(f"[{row['n']}] {row['kind']} {row['target']}: {row['result']}"
                               + (f" ({row['reason']})" if row.get('reason') else '')
                               + f" after {row['duration_s']:.1f} s")

    def job(self, kind, target, x, y, yaw, interruptible=True, timeout=None, retarget=None, check=None):
        row = {'kind': kind, 'target': target, 'goal_x': round(x, 3), 'goal_y': round(y, 3),
               'goal_yaw': round(yaw, 4), 't_start': round(self.now(), 2), **self.where()}
        path0 = self.odometer
        outcome, status, error = self.navigate(x, y, yaw, timeout or self.p['task_timeout'],
                                               interruptible, retarget)
        row.update(t_end=round(self.now(), 2), nav2_status=status, nav2_error=error,
                   path_m=round(self.odometer - path0, 3))
        row['duration_s'] = round(row['t_end'] - row['t_start'], 2)
        if self.robot:
            row.update(end_x=round(self.robot[0], 3), end_y=round(self.robot[1], 3),
                       err_xy_m=round(math.hypot(self.robot[0] - x, self.robot[1] - y), 3),
                       err_yaw_rad=round(abs(wrap(self.robot[2] - yaw)), 4))
        if outcome == 'succeeded':
            reason = check(row) if check else ''
            row.update(result='fail' if reason else 'success', reason=reason)
        else:
            row.update(result='interrupted' if outcome == 'interrupted' else 'fail',
                       reason='' if outcome == 'interrupted' else
                       {'aborted': 'nav2_aborted', 'canceled': 'nav2_canceled'}.get(outcome, outcome))
        self.log(row)
        return row

    # Jobs --------------------------------------------------------------------------------

    def facing(self, x, y):
        """Yaw from the robot to (x, y), so it does not spin on the spot at the goal."""
        return math.atan2(y - self.robot[1], x - self.robot[0]) if self.robot else 0.0

    def patrol_job(self, goal):
        return self.job('patrol', goal['room'], goal['x'], goal['y'], self.facing(goal['x'], goal['y']))

    def room_job(self, name):
        goal = max((g for g in self.home.patrol if g['room'] == name), key=lambda g: g['clearance'])
        return self.job('room', name, goal['x'], goal['y'], self.facing(goal['x'], goal['y']))

    def handover_spot(self):
        """A free spot approach_distance from the resident, nearest the robot, facing them."""
        if not self.resident or not self.robot:
            return None
        rx, ry = self.resident
        d, best = self.p['approach_distance'], None
        for i in range(24):
            a = 2 * math.pi * i / 24
            x, y = rx + d * math.cos(a), ry + d * math.sin(a)
            if self.home.fits(x, y, self.p['robot_radius'] + 0.05):
                dist = math.hypot(x - self.robot[0], y - self.robot[1])
                if best is None or dist < best[0]:
                    best = (dist, x, y, math.atan2(ry - y, rx - x))
        return best[1:] if best else None

    def resident_job(self):
        spot = self.handover_spot()
        if spot is None:
            row = {'kind': 'resident', 'target': 'resident', 'goal_x': '', 'goal_y': '', 'goal_yaw': '',
                   't_start': round(self.now(), 2), 't_end': round(self.now(), 2), 'duration_s': 0.0,
                   'result': 'fail', 'reason': 'no_free_spot_near_resident' if self.resident else 'resident_unknown',
                   **self.where()}
            self.log(row)
            self.wait(5.0)
            return row
        aimed_at = [self.resident]

        def retarget():
            # Follow the resident when they walk on; small moves are not worth a new plan.
            if self.resident and math.hypot(self.resident[0] - aimed_at[0][0],
                                            self.resident[1] - aimed_at[0][1]) > 0.75:
                aimed_at[0] = self.resident
                return self.handover_spot()
            return None

        def check(row):
            if not self.resident:
                return 'resident_unknown'
            rx, ry = self.resident
            x, y, yaw = self.robot
            if math.hypot(rx - x, ry - y) > self.p['approach_distance'] + 0.5:
                return 'resident_moved_away'
            if abs(wrap(math.atan2(ry - y, rx - x) - yaw)) > math.radians(30):
                return 'not_facing_resident'
            return ''
        return self.job('resident', 'resident', *spot, retarget=retarget, check=check)

    def return_to_dock(self):
        d = self.home.dock

        def check(row):
            if row['err_xy_m'] > self.p['dock_xy_tol'] or row['err_yaw_rad'] > self.p['dock_yaw_tol']:
                return 'off_dock'    # Nav2 believed it arrived; AMCL was off
            return ''
        self.n_returns += 1
        row = self.job('dock', d.get('room', 'dock'), d['x'], d['y'], d['yaw'], interruptible=False,
                       timeout=self.p['dock_timeout'], check=check)
        self.wait(self.p['rest'])
        self.schedule_call()
        return row

    # Run ---------------------------------------------------------------------------------

    def write_record(self):
        out = os.path.join(os.path.expanduser(self.p['log_dir']), self.run_id)
        os.makedirs(out, exist_ok=True)
        self.csv_path = os.path.join(out, 'events.csv')
        with open(self.csv_path, 'w', newline='') as f:
            csv.DictWriter(f, FIELDS).writeheader()
        record = {'run_id': self.run_id, 'started_wall': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                  'started_sim_s': round(self.now(), 2), 'ros_distro': os.environ.get('ROS_DISTRO', ''),
                  'parameters': self.p, 'dock': self.home.dock,
                  'unreachable_rooms': self.home.ann.get('unreachable_rooms') or [],
                  'home_files_sha256': self.home.checksums()}
        with open(os.path.join(out, 'run.yaml'), 'w') as f:
            yaml.safe_dump(record, f, sort_keys=False)
        self.get_logger().info(f'Logging to {out}')

    def run(self):
        self.get_logger().info('Waiting for /clock, /ground_truth/odom and Nav2...')
        while rclpy.ok() and (self.now() <= 0.0 or self.robot is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.client.wait_for_server()
        if self.p['task'] in ('resident', 'mixed'):
            end = self.now() + 10.0
            while rclpy.ok() and self.resident is None and self.now() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.resident is None:
                self.get_logger().warn('No /people/odom yet: is navigation.launch.py running with world:=?')
        self.write_record()

        self.n_returns, start = 0, self.now()
        self.schedule_call()
        tour, done_once = [], False
        while rclpy.ok():
            if self.p['duration'] and self.now() - start >= self.p['duration']:
                break
            if self.p['returns'] and self.n_returns >= self.p['returns']:
                break
            if self.call_due():
                self.return_to_dock()
                done_once = False     # back to the job it was called away from
                continue
            task = self.p['task']
            if done_once and task in ('room', 'resident') and not self.p['returns']:
                break
            if task == 'patrol':
                if not tour:
                    tour = self.nearest_first(self.home.patrol)
                self.patrol_job(tour.pop(0))
            elif task == 'room':
                if done_once:
                    self.wait(1.0)        # stay in the room until the dock call
                else:
                    self.room_job(self.p['room'])
            elif task == 'resident':
                if done_once:
                    self.wait(self.p['resident_wait'])
                self.resident_job()
            else:
                if self.rng.random() < 0.5:
                    self.resident_job()
                else:
                    self.patrol_job(self.rng.choice(self.home.patrol))
            done_once = True
        self.get_logger().info(f'Done: {self.n} events logged to {self.csv_path}')

    def nearest_first(self, goals):
        left, order = list(goals), []
        x, y = self.robot[:2]
        while left:
            g = min(left, key=lambda g: math.hypot(g['x'] - x, g['y'] - y))
            left.remove(g)
            order.append(g)
            x, y = g['x'], g['y']
        return order


def main():
    rclpy.init()
    node = TaskRunner()
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
