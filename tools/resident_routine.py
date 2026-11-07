#!/usr/bin/env python3
"""
Give a converted home a resident who walks a daily routine between its rooms.

  python3 tools/resident_routine.py homes/guest_flat
  python3 tools/resident_routine.py homes/guest_flat --dwell 45 --speed 0.6

Reads the home's annotations.yaml, rooms.pgm and map.pgm (written by scan_to_world.py,
which also calls this at the end) and writes <actor name='resident'> into the home's
world file, replacing any earlier one. people_publisher.py reads the same actor, so
/people/odom follows it. Also writes routine.png: the walking path (orange) and the
stops (purple) on the map.

The routine is a compressed day, looped: bedroom -> bathroom -> each day room (kitchen,
living room, other rooms; halls only if there is nothing else) -> bathroom -> first day
room -> back to bed, with a pause at each stop. It walks the real doorways, keeps clear of walls and furniture, and
never stops near the robot's spawn point or dock. Standard library only.
"""
import argparse
import heapq
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scan_to_world import GRID, PREVIEW_SCALE, clearance, write_png  # noqa: E402

# Below the 0.8 m/s gait speed that marks frailty in older adults (Studenski et al. 2011,
# JAMA), as expected of residents who need care; healthy people in their 70s walk at
# about 1.1-1.3 m/s (Bohannon 1997, Age and Ageing). Verify both before citing.
SPEED = 0.65
TURN_RATE = 1.5        # rad/s when turning on the spot
PERSON_RADIUS = 0.25   # half a person's shoulder width plus a margin, in metres
KEEP_AWAY = 1.2        # no stop within this distance of the robot's spawn point or dock
ACTOR_Z = 1.0          # the walk.dae skeleton's origin is at hip height

CATEGORIES = (('bedroom', ('bed',)), ('bathroom', ('bath', 'wc', 'toilet', 'shower')),
              ('kitchen', ('kitchen',)), ('living', ('living', 'lounge', 'sitting', 'dining')),
              ('hall', ('hall', 'corridor', 'landing', 'entrance')),
              ('skip', ('closet', 'cupboard', 'storage', 'utility', 'unlabelled')))

ACTOR = """    <actor name='resident'>
      <skin>
        <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
      </skin>
      <animation name='walk'>
        <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id='0' type='walk'>
{waypoints}        </trajectory>
      </script>
    </actor>
"""


def read_annotations(path):
    """The flow-style YAML scan_to_world.py writes: top-level keys holding one {..} per line."""
    item = re.compile(r'(\w+): ("[^"]*"|\[[^\]]*\]|[^,}]+)')

    def value(v):
        v = v.strip()
        if v.startswith('"'):
            return v[1:-1]
        if v.startswith('['):
            return [value(x) for x in v[1:-1].split(',') if x.strip()]
        try:
            return float(v)
        except ValueError:
            return None if v == 'null' else v

    data, key = {}, None
    for line in open(path):
        line = line.split('  #')[0].rstrip()
        if not line or line.startswith('#'):
            continue
        m = re.match(r'^(\w+):\s*(.*)$', line)
        if m:
            key, rest = m.groups()
            data[key] = {k: value(v) for k, v in item.findall(rest)} if rest.startswith('{') else []
        elif line.lstrip().startswith('- {') and key:
            data[key].append({k: value(v) for k, v in item.findall(line)})
    return data


def read_pgm(path):
    with open(path, 'rb') as f:
        raw = f.read()
    header, pos = [], 0
    while len(header) < 4:
        end = raw.index(b'\n', pos)
        header += raw[pos:end].split()
        pos = end + 1
    w, h = int(header[1]), int(header[2])
    px = raw[pos:pos + w * h]
    # PGM rows run from the top (+y); flip so cell k = y * w + x with y up, like the converter
    return w, h, bytes(b for y in range(h - 1, -1, -1) for b in px[y * w:(y + 1) * w])


def category(name):
    low = name.lower()
    for cat, words in CATEGORIES:
        if any(w in low for w in words):
            return cat
    return 'other'


def plan_path(start, goal, ok, cost, nx, ny):
    """Cheapest 8-connected path over walkable cells, preferring the middle of free space."""
    dist, prev = {start: 0.0}, {}
    heap = [(0.0, start)]
    while heap:
        d, k = heapq.heappop(heap)
        if k == goal:
            break
        if d > dist[k]:
            continue
        x, y = k % nx, k // nx
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            xx, yy = x + dx, y + dy
            if not (0 <= xx < nx and 0 <= yy < ny):
                continue
            kk = yy * nx + xx
            if not ok[kk]:
                continue
            nd = d + math.hypot(dx, dy) * cost[kk]
            if nd < dist.get(kk, math.inf):
                dist[kk], prev[kk] = nd, k
                heapq.heappush(heap, (nd, kk))
    if goal not in dist:
        return None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return path[::-1]


def shortcut(path, ok, nx):
    """Keep only the corners: jump ahead while the straight line stays on walkable cells."""
    def clear_line(a, b):
        ax, ay, bx, by = a % nx, a // nx, b % nx, b // nx
        n = max(abs(bx - ax), abs(by - ay)) * 2 or 1
        return all(ok[round(ay + (by - ay) * i / n) * nx + round(ax + (bx - ax) * i / n)] for i in range(n + 1))
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not clear_line(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def build(home, speed=SPEED, dwell=30.0):
    name = os.path.basename(os.path.normpath(home))
    ann = read_annotations(os.path.join(home, 'annotations.yaml'))
    nx, ny, mp = read_pgm(os.path.join(home, 'map.pgm'))
    _, _, rooms = read_pgm(os.path.join(home, 'rooms.pgm'))
    x0, y0 = ann['grid']['origin']

    def centre(k):
        return x0 + (k % nx + 0.5) * GRID, y0 + (k // nx + 0.5) * GRID

    clear = clearance([v != 254 for v in mp], nx, ny)
    ok = [v == 254 and c >= PERSON_RADIUS for v, c in zip(mp, clear)]
    cost = [1.0 + 0.3 / max(c, GRID) for c in clear]

    avoid = [(p['x'], p['y']) for p in (ann.get('spawn'), ann.get('dock')) if isinstance(p, dict) and p]
    stops = {}
    for r in ann['rooms']:
        cat = category(r['name'])
        if cat == 'skip':
            continue
        cells = [k for k in range(nx * ny) if rooms[k] == int(r['id']) and ok[k]
                 and all(math.hypot(centre(k)[0] - ax, centre(k)[1] - ay) >= KEEP_AWAY for ax, ay in avoid)]
        if cells:
            stops[r['name']] = (cat, max(cells, key=lambda k: clear[k]))

    def first(cat):
        return [n for n, (c, _) in stops.items() if c == cat]
    day = first('kitchen') + first('living') + first('other') or first('hall')
    bed, bath = first('bedroom')[:1], first('bathroom')[:1]
    order = bed + bath + day + bath + day[:1] + bed
    order = [n for i, n in enumerate(order) if i == 0 or n != order[i - 1]]
    if order and order[0] == order[-1]:
        order.pop()
    if len(order) < 2:
        order = list(stops)
    if len(order) < 2:
        raise SystemExit(f'{name}: fewer than two rooms the resident can stand in; no routine written.')

    t, waypoints, path_cells = 0.0, [], []
    x, y = centre(stops[order[0]][1])
    yaw = 0.0
    waypoints.append((t, x, y, yaw))
    for a, b in zip(order, order[1:] + order[:1]):
        t += dwell
        waypoints.append((t, x, y, yaw))
        path = plan_path(stops[a][1], stops[b][1], ok, cost, nx, ny)
        if path is None:
            raise SystemExit(f'{name}: no walkable path from {a} to {b}.')
        path_cells += path
        for k in shortcut(path, ok, nx)[1:]:
            nxp, nyp = centre(k)
            heading = math.atan2(nyp - y, nxp - x)
            turn = abs(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
            if turn > 0.05:
                t += turn / TURN_RATE
                yaw = yaw + math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
                waypoints.append((t, x, y, yaw))
            t += math.hypot(nxp - x, nyp - y) / speed
            x, y = nxp, nyp
            waypoints.append((t, x, y, yaw))
    back = math.atan2(math.sin(-yaw), math.cos(-yaw))
    if abs(back) > 0.05:
        t += abs(back) / TURN_RATE
        yaw += back
        waypoints.append((t, x, y, yaw))

    xml = ''.join(f'          <waypoint><time>{wt:.2f}</time>'
                  f'<pose>{wx:.3f} {wy:.3f} {ACTOR_Z} 0 0 {math.atan2(math.sin(wa), math.cos(wa)):.4f}</pose>'
                  '</waypoint>\n'
                  for wt, wx, wy, wa in waypoints)
    world = os.path.join(home, f'{name}.world')
    text = open(world).read()
    text = re.sub(r"\s*<actor name='resident'>.*?</actor>\n?", '\n', text, flags=re.S)
    text = text.replace('  </world>', ACTOR.format(waypoints=xml) + '  </world>')
    with open(world, 'w') as f:
        f.write(text)

    on_path = set(path_cells)
    stop_cells = {stops[n][1] for n in order}
    rows = []
    for yy in range(ny - 1, -1, -1):
        row = []
        for xx in range(nx):
            k = yy * nx + xx
            near_stop = any(abs(xx - s % nx) <= 1 and abs(yy - s // nx) <= 1 for s in stop_cells)
            row += ((120, 40, 160) if near_stop else (230, 120, 20) if k in on_path else
                    (30, 30, 30) if mp[k] == 0 else (250, 250, 250) if mp[k] == 254 else (200, 200, 200))
        rows += [[c for px in zip(row[0::3], row[1::3], row[2::3]) for c in px * PREVIEW_SCALE]] * PREVIEW_SCALE
    write_png(os.path.join(home, 'routine.png'), rows)

    walked = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(waypoints, waypoints[1:]))
    print(f'Resident routine ({name}): ' + ' -> '.join(order) + f' -> {order[0]}, looped')
    print(f'  {len(waypoints)} waypoints, {walked:.1f} m walked per loop at {speed} m/s, '
          f'{dwell:.0f} s at each stop; loop length {t:.0f} s')
    return t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('home', help='a folder written by scan_to_world.py, e.g. homes/guest_flat')
    ap.add_argument('--speed', type=float, default=SPEED, help=f'walking speed in m/s (default {SPEED})')
    ap.add_argument('--dwell', type=float, default=30.0, help='seconds spent at each stop (default 30)')
    args = ap.parse_args()
    build(args.home, args.speed, args.dwell)


if __name__ == '__main__':
    main()
