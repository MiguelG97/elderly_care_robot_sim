#!/usr/bin/env python3
"""
analyze_runs: compare navigation runs recorded with ros2 bag.

  python3 analyze_runs.py recordings/baseline_run1 recordings/social_run1

For each bag (needs /ground_truth/odom and /people/odom) it reports, using
simulation time from the message headers:
  - time moving   : from the robot starting to move until it stops for good
  - path length   : distance the robot actually drove
  - min distance  : closest centre-to-centre distance between robot and person
  - time < 1.2 m  : how long the robot was within 1.2 m of the person
                    (the outer edge of the "personal" zone in Hall's proxemics)
  - time < 0.8 m  : how long it was uncomfortably close
It also writes <bag>_distance.csv and, if matplotlib is installed, a figure
comparing the runs (comparison.png next to the first bag).
"""
import bisect
import csv
import math
import os
import sys


def read_bag(path):
    """Return (robot, person) lists of (t, x, y, speed) from a ros2 bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    opened = False
    for storage in ('', 'mcap', 'sqlite3'):
        try:
            reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                        rosbag2_py.ConverterOptions('', ''))
            opened = True
            break
        except Exception:
            reader = rosbag2_py.SequentialReader()
    if not opened:
        raise RuntimeError(f'Could not open bag {path}')
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    robot, person = [], []
    for name in ('/ground_truth/odom', '/people/odom'):
        if name not in types:
            raise RuntimeError(f'{path} has no {name} topic (recorded topics: {sorted(types)})')
    odom_type = get_message(types['/ground_truth/odom'])
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in ('/ground_truth/odom', '/people/odom'):
            continue
        m = deserialize_message(data, odom_type)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        (robot if topic == '/ground_truth/odom' else person).append((t, p.x, p.y, v))
    robot.sort()
    person.sort()
    return robot, person


def person_at(person, t):
    """Linearly interpolate the person's (x, y) at time t."""
    times = [p[0] for p in person]
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return person[0][1:3]
    if i >= len(person):
        return person[-1][1:3]
    t0, x0, y0, _ = person[i - 1]
    t1, x1, y1, _ = person[i]
    s = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    return x0 + s * (x1 - x0), y0 + s * (y1 - y0)


def analyse(robot, person, moving_speed=0.03):
    moving = [i for i, r in enumerate(robot) if r[3] > moving_speed]
    if not moving or not person:
        raise RuntimeError('Robot never moved, or no person data recorded')
    a, b = moving[0], moving[-1]
    seg = robot[a:b + 1]
    length = sum(math.hypot(q[1] - p[1], q[2] - p[2]) for p, q in zip(seg, seg[1:]))
    rows, close12, close08 = [], 0.0, 0.0
    for p, q in zip(seg, seg[1:]):
        px, py = person_at(person, p[0])
        d = math.hypot(p[1] - px, p[2] - py)
        dt = q[0] - p[0]
        close12 += dt if d < 1.2 else 0.0
        close08 += dt if d < 0.8 else 0.0
        rows.append((p[0] - seg[0][0], p[1], p[2], px, py, d))
    return {
        'time_moving_s': seg[-1][0] - seg[0][0],
        'path_length_m': length,
        'min_distance_m': min(r[5] for r in rows),
        'time_within_1.2m_s': close12,
        'time_within_0.8m_s': close08,
        'rows': rows,
    }


def main(paths):
    results = {}
    for path in paths:
        robot, person = read_bag(path)
        res = analyse(robot, person)
        results[os.path.basename(os.path.normpath(path))] = res
        with open(os.path.normpath(path) + '_distance.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'robot_x', 'robot_y', 'person_x', 'person_y', 'distance_m'])
            w.writerows([[round(v, 3) for v in r] for r in res['rows']])

    keys = [('time_moving_s', 'Time moving (s)'), ('path_length_m', 'Path length (m)'),
            ('min_distance_m', 'Min distance to person (m)'),
            ('time_within_1.2m_s', 'Time within 1.2 m (s)'),
            ('time_within_0.8m_s', 'Time within 0.8 m (s)')]
    names = list(results)
    print('| Metric | ' + ' | '.join(names) + ' |')
    print('|---|' + '---|' * len(names))
    for k, label in keys:
        print(f'| {label} | ' + ' | '.join(f'{results[n][k]:.2f}' for n in names) + ' |')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('\n(matplotlib not installed: skipped the figure. sudo apt install python3-matplotlib)')
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
    for n in names:
        rows = results[n]['rows']
        ax1.plot([r[1] for r in rows], [r[2] for r in rows], label=f'robot: {n}')
        ax2.plot([r[0] for r in rows], [r[5] for r in rows], label=n)
    rows = results[names[0]]['rows']
    ax1.plot([r[3] for r in rows], [r[4] for r in rows], ':', color='grey', label='person')
    ax1.set_xlabel('x (m)'); ax1.set_ylabel('y (m)'); ax1.set_aspect('equal'); ax1.legend(fontsize=8)
    ax1.set_title('Paths in the corridor')
    for y, style in ((1.2, '--'), (0.8, ':')):
        ax2.axhline(y, color='grey', linestyle=style, linewidth=1)
    ax2.set_xlabel('time since robot started moving (s, sim time)')
    ax2.set_ylabel('robot-person distance (m)'); ax2.legend(fontsize=8)
    ax2.set_title('Distance to the person (dashed: 1.2 m, dotted: 0.8 m)')
    fig.tight_layout()
    out = os.path.join(os.path.dirname(os.path.normpath(paths[0])), 'comparison.png')
    fig.savefig(out, dpi=150)
    print(f'\nFigure saved: {out}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1:])