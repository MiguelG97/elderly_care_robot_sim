#!/usr/bin/env python3
"""
Turn a Polycam Room-mode scan (OBJ export, zipped or not) into a Gazebo model and world.

  python3 tools/scan_to_world.py "Scans/Guest_Flat/Flat/OBJ/Main Guest Flat.zip" guest_flat \
    --floorplan "Scans/Guest_Flat/Flat/Floorplan/Main Guest Flat.csv"

Writes homes/<name>/ (git-ignored, the homes are private):
  model.sdf, model.config      static model of the home
  meshes/visual.obj + .mtl     everything except the door panels, so doorways are open
  meshes/walls.obj             walls only, used as the collision mesh
  <name>.world                 the world to launch
  map.pgm, map.yaml            Nav2 map in Gazebo world coordinates, for navigation.launch.py
  annotations.yaml, rooms.pgm  rooms (named from the floor-plan CSV given with --floorplan),
                               doorways, narrow passages, a suggested dock pose and resident
                               spots; rooms.pgm holds each cell's room id on the map's grid
  annotations.png              the same, to check by eye
  preview.png                  top view: obstacles (black), doorways (blue), floor the robot's
                               centre can reach (green), other floor inside (white), outside
                               (grey), spawn point (red)

Then, after colcon build, the script's printed commands start the home and Nav2 in it. It needs only the Python standard library.

What the scan looks like (Room mode, RoomPlan underneath): metres, Y-up, centred on the
origin, one material per element. Walls, doors and windows are a few flat quads; doors
and windows are single two-triangle panels; furniture is a detailed mesh; the floor is a
10 cm slab. Element types are not named, so they are told apart by shape.
"""
import argparse
import csv
import itertools
import math
import os
import struct
import sys
import zipfile
import zlib
from collections import deque

GRID = 0.05            # m per preview cell, same as the Nav2 maps
ROBOT_RADIUS = 0.275   # TIAGo PMB2, from tiago_2dnav (same as nav2_params.yaml)
ROBOT_HEIGHT = 1.2     # anything lower than this can hit TIAGo's base, torso or head
PREVIEW_SCALE = 4
# Height band drawn into the Nav2 map: the base laser's plane is low in the PMB2 base, so
# AMCL must match what it sees there (table legs, not table tops).
MAP_BAND = (0.05, 0.40)
NARROW = 1.2          # passages narrower than this are where a robot and a person cannot pass
                      # each other (Hall's personal-distance limit)
FLOOR_FURNITURE = 0.3  # furniture starting higher than this sits on something else or hangs on a wall


def read_scan(path):
    """Return (obj text, mtl text) from a .obj file or a zip holding one."""
    if path.lower().endswith('.zip'):
        with zipfile.ZipFile(path) as z:
            names = z.namelist()
            obj = next(n for n in names if n.lower().endswith('.obj'))
            mtl = next((n for n in names if n.lower().endswith('.mtl')), None)
            return (z.read(obj).decode(errors='replace'),
                    z.read(mtl).decode(errors='replace') if mtl else '')
    mtl_path = os.path.splitext(path)[0] + '.mtl'
    mtl = open(mtl_path, errors='replace').read() if os.path.exists(mtl_path) else ''
    return open(path, errors='replace').read(), mtl


def parse(obj_text, mtl_text):
    colours, name = {}, None
    for line in mtl_text.splitlines():
        p = line.split()
        if p and p[0] == 'newmtl':
            name = line.split(None, 1)[1].strip()
        elif p and p[0] == 'Kd' and name:
            colours[name] = tuple(float(c) for c in p[1:4])

    verts, groups = [], []
    for line in obj_text.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == 'v':
            x, y, z = map(float, p[1:4])
            verts.append((x, -z, y))          # Y-up scan -> Z-up Gazebo
        elif p[0] == 'usemtl':
            mat = line.split(None, 1)[1].strip()
            groups.append({'mat': mat, 'colour': colours.get(mat, (0.7, 0.7, 0.7)), 'tris': []})
        elif p[0] == 'f':
            if not groups:
                groups.append({'mat': 'default', 'colour': (0.7, 0.7, 0.7), 'tris': []})
            idx = [int(t.split('/')[0]) for t in p[1:]]
            idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
            for k in range(1, len(idx) - 1):
                groups[-1]['tris'].append((idx[0], idx[k], idx[k + 1]))
    return verts, [g for g in groups if g['tris']]


def bounds(verts, tris):
    pts = [verts[i] for t in tris for i in t]
    lo = [min(p[k] for p in pts) for k in range(3)]
    hi = [max(p[k] for p in pts) for k in range(3)]
    return lo, hi


def classify(verts, groups):
    for g in groups:
        g['lo'], g['hi'] = bounds(verts, g['tris'])
        g['size'] = [g['hi'][k] - g['lo'][k] for k in range(3)]

    slab = max((g for g in groups if g['size'][2] <= 0.15),
               key=lambda g: (round(g['size'][0] * g['size'][1], 1), g['size'][2]))
    floor_z = slab['hi'][2]
    wall_top = max(g['hi'][2] for g in groups)

    for g in groups:
        sx, sy, sz = g['size']
        bottom = g['lo'][2] - floor_z
        panel = len(g['tris']) == 2 and min(sx, sy) < 0.01 and sz > 0.3
        if g is slab or (sz < 0.01 and abs(g['lo'][2] - floor_z) < 0.01 and sx * sy > 0.5 * slab['size'][0] * slab['size'][1]):
            g['kind'] = 'floor'
        elif panel and bottom < 0.05 and g['hi'][2] < wall_top - 0.05:
            g['kind'] = 'door'
        elif panel and (bottom > 0.05 or g['hi'][2] < wall_top - 0.05):
            g['kind'] = 'window'
        elif len(g['tris']) <= 12:
            g['kind'] = 'wall'
        elif bottom < FLOOR_FURNITURE:
            g['kind'] = 'furniture'
        else:
            g['kind'] = 'raised'      # wall cabinet, item on a table: visual only
    return slab, floor_z


def footprint_box(pts):
    """Smallest rectangle around the points seen from above: (cx, cy, length, width, yaw).
    Furniture is often turned, and an axis-aligned box would stick out past it."""
    pts = sorted(set((round(x, 4), round(y, 4)) for x, y in pts))
    if len(pts) < 3:
        (x, y), = pts[:1] or [(0.0, 0.0)]
        return x, y, 0.0, 0.0, 0.0

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    best = None
    for i in range(len(hull)):
        (ax, ay), (bx, by) = hull[i], hull[(i + 1) % len(hull)]
        yaw = math.atan2(by - ay, bx - ax)
        c, s = math.cos(yaw), math.sin(yaw)
        us = [x * c + y * s for x, y in hull]
        vs = [-x * s + y * c for x, y in hull]
        area = (max(us) - min(us)) * (max(vs) - min(vs))
        if best is None or area < best[0]:
            mu, mv = (max(us) + min(us)) / 2, (max(vs) + min(vs)) / 2
            best = (area, mu * c - mv * s, mu * s + mv * c, max(us) - min(us), max(vs) - min(vs), yaw)
    return best[1:]


def write_obj(path, verts, groups, floor_z, mtl_name=None, skip_floor_faces=False):
    used, out_v, lines = {}, [], []
    if mtl_name:
        lines.append(f'mtllib {mtl_name}')
    normals = []
    for gi, g in enumerate(groups):
        lines.append(f'o part_{gi}')
        if mtl_name:
            lines.append(f'usemtl m{gi}')
        for t in g['tris']:
            if skip_floor_faces and max(verts[i][2] for i in t) - floor_z < 0.02:
                continue
            for i in t:
                if i not in used:
                    used[i] = len(out_v) + 1
                    x, y, z = verts[i]
                    out_v.append(f'v {x:.4f} {y:.4f} {z - floor_z:.4f}')
            a, b, c = (verts[i] for i in t)
            u = [b[k] - a[k] for k in range(3)]
            w = [c[k] - a[k] for k in range(3)]
            n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
            ln = math.sqrt(sum(v * v for v in n)) or 1.0
            normals.append(f'vn {n[0] / ln:.4f} {n[1] / ln:.4f} {n[2] / ln:.4f}')
            ni = len(normals)
            lines.append('f ' + ' '.join(f'{used[i]}//{ni}' for i in t))
    with open(path, 'w') as f:
        f.write('\n'.join(out_v + normals + lines) + '\n')
    if mtl_name:
        with open(os.path.join(os.path.dirname(path), mtl_name), 'w') as f:
            for gi, g in enumerate(groups):
                r, gr, b = g['colour']
                f.write(f'newmtl m{gi}\nKa {r:.3f} {gr:.3f} {b:.3f}\nKd {r:.3f} {gr:.3f} {b:.3f}\nKs 0 0 0\nd 1\nillum 1\n')


def occupancy(verts, groups, floor_z, x0, y0, nx, ny, kinds=('wall', 'furniture', 'raised'),
              band=(0.02, ROBOT_HEIGHT)):
    """Cells holding a surface of the given kinds between the floor and robot height."""
    occ = bytearray(nx * ny)
    step = GRID / 2
    for g in groups:
        if g['kind'] not in kinds:
            continue
        for t in g['tris']:
            a, b, c = (verts[i] for i in t)
            n = max(1, int(max(math.dist(a, b), math.dist(b, c), math.dist(c, a)) / step))
            for i in range(n + 1):
                for j in range(n + 1 - i):
                    u, v = i / n, j / n
                    z = a[2] + u * (b[2] - a[2]) + v * (c[2] - a[2]) - floor_z
                    if not band[0] < z < band[1]:
                        continue
                    x = a[0] + u * (b[0] - a[0]) + v * (c[0] - a[0])
                    y = a[1] + u * (b[1] - a[1]) + v * (c[1] - a[1])
                    cx, cy = int((x - x0) / GRID), int((y - y0) / GRID)
                    if 0 <= cx < nx and 0 <= cy < ny:
                        occ[cy * nx + cx] = 1
    return occ


def clearance(occ, nx, ny):
    """Approximate distance in metres from each cell to the nearest obstacle (chamfer 3-4)."""
    big = 10 ** 9
    d = [0 if o else big for o in occ]
    passes = ((range(ny), range(nx), ((-1, 0, 3), (0, -1, 3), (-1, -1, 4), (1, -1, 4))),
              (range(ny - 1, -1, -1), range(nx - 1, -1, -1), ((1, 0, 3), (0, 1, 3), (1, 1, 4), (-1, 1, 4))))
    for rows, cols, nbrs in passes:
        for y in rows:
            for x in cols:
                k = y * nx + x
                for dx, dy, w in nbrs:
                    xx, yy = x + dx, y + dy
                    if 0 <= xx < nx and 0 <= yy < ny and d[yy * nx + xx] + w < d[k]:
                        d[k] = d[yy * nx + xx] + w
    return [v * GRID / 3 for v in d]


def close_gaps(occ, nx, ny, cells=1):
    """Fill gaps up to 2 * cells wide, such as the inside of a wall drawn as its two faces."""
    def grow(src, value):
        out = bytearray(src)
        for k in range(nx * ny):
            if src[k] == value:
                x, y = k % nx, k // nx
                for xx in (x - 1, x, x + 1):
                    for yy in (y - 1, y, y + 1):
                        if 0 <= xx < nx and 0 <= yy < ny:
                            out[yy * nx + xx] = value
        return out
    for _ in range(cells):
        occ = grow(occ, 1)
    for _ in range(cells):
        occ = grow(occ, 0)
    return occ


def flood(ok, starts, nx):
    seen = bytearray(len(ok))
    q = deque(k for k in starts if ok[k])
    for k in q:
        seen[k] = 1
    while q:
        k = q.popleft()
        x, y = k % nx, k // nx
        for xx, yy in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            kk = yy * nx + xx
            if 0 <= xx < nx and 0 <= kk < len(ok) and ok[kk] and not seen[kk]:
                seen[kk] = 1
                q.append(kk)
    return seen


def largest_region(ok, nx, ny):
    seen, best = bytearray(nx * ny), []
    for start in range(nx * ny):
        if not ok[start] or seen[start]:
            continue
        region, q = [], deque([start])
        seen[start] = 1
        while q:
            k = q.popleft()
            region.append(k)
            x, y = k % nx, k // nx
            for xx, yy in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                kk = yy * nx + xx
                if 0 <= xx < nx and 0 <= yy < ny and ok[kk] and not seen[kk]:
                    seen[kk] = 1
                    q.append(kk)
        if len(region) > len(best):
            best = region
    return best


def write_png(path, rgb_rows):
    h, w = len(rgb_rows), len(rgb_rows[0]) // 3
    raw = b''.join(b'\x00' + bytes(r) for r in rgb_rows)

    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def read_floorplan(path):
    """Room name -> {'area', 'dims'} from a RoomPlan floor-plan CSV. Only the per-room area
    and dimension rows are read: the file's 'Name' row can hold the home's address."""
    rooms = {}
    with open(path, encoding='utf-8-sig') as f:
        for row in csv.reader(f):
            if len(row) < 3 or row[0] in ('Room', 'Entire Roomplan'):
                continue
            room, quantity, value = row[0], row[1], row[2]
            if quantity == 'Floor area [m^2]':
                rooms.setdefault(room, {})['area'] = float(value)
            elif quantity in ('Dimensions [m]', 'Inscribed dimensions [m]'):
                try:
                    rooms.setdefault(room, {})['dims'] = sorted(float(v) for v in value.lower().split('x'))
                except ValueError:
                    pass
    return {name: r for name, r in rooms.items() if 'area' in r}


def enclosed(barrier, nx, ny, need=6):
    """Cells with a wall somewhere in at least `need` of the 8 compass directions. Unlike a
    flood from the edge, this keeps a room whose scan is missing a stretch of wall."""
    n = nx * ny
    hits = [0] * n
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        seen = bytearray(n)
        xs = range(nx - 1, -1, -1) if dx > 0 else range(nx)
        ys = range(ny - 1, -1, -1) if dy > 0 else range(ny)
        for y in ys:
            for x in xs:
                k = y * nx + x
                xx, yy = x + dx, y + dy
                ahead = 0 <= xx < nx and 0 <= yy < ny and seen[yy * nx + xx]
                if ahead:
                    hits[k] += 1
                seen[k] = barrier[k] or ahead
    return [h >= need for h in hits]


def components(ok, nx, ny, diagonal=False):
    steps = [(1, 0), (-1, 0), (0, 1), (0, -1)] + ([(1, 1), (1, -1), (-1, 1), (-1, -1)] if diagonal else [])
    label, regions = [0] * (nx * ny), []
    for start in range(nx * ny):
        if not ok[start] or label[start]:
            continue
        regions.append([start])
        label[start] = len(regions)
        q = deque([start])
        while q:
            k = q.popleft()
            x, y = k % nx, k // nx
            for dx, dy in steps:
                xx, yy = x + dx, y + dy
                kk = yy * nx + xx
                if 0 <= xx < nx and 0 <= yy < ny and ok[kk] and not label[kk]:
                    label[kk] = len(regions)
                    regions[-1].append(kk)
                    q.append(kk)
    return label, regions


def room_shapes(regions, barrier, nx):
    """(floor area, sorted bounding-box sides) per room. The wall cells along a room's edge
    are only partly wall, so half of each counts as floor (RoomPlan measures to the wall)."""
    shapes = []
    for cells in regions:
        inside = set(cells)
        edge = {n for k in cells for n in (k - 1, k + 1, k - nx, k + nx)
                if n not in inside and 0 <= n < len(barrier) and barrier[n]}
        xs, ys = [k % nx for k in cells], [k // nx for k in cells]
        shapes.append(((len(cells) + 0.5 * len(edge)) * GRID * GRID,
                       sorted(((max(xs) - min(xs) + 2) * GRID, (max(ys) - min(ys) + 2) * GRID))))
    return shapes


def name_rooms(shapes, plan):
    """Match the scan's rooms to the floor plan's by area and size; unmatched ones get 'Room n'."""
    names = list(plan)

    def cost(i, name):
        area, dims = shapes[i]
        c = abs(math.log(area / plan[name]['area']))
        if 'dims' in plan[name]:
            c += sum(abs(a - b) for a, b in zip(dims, plan[name]['dims'])) / sum(plan[name]['dims'])
        return c

    best = {}
    small, large = (names, list(range(len(shapes)))) if len(names) <= len(shapes) else (list(range(len(shapes))), names)
    if math.perm(len(large), len(small)) <= 200000:
        top = None
        for pick in itertools.permutations(large, len(small)):
            pairs = list(zip(small, pick))
            total = sum(cost(*((b, a) if isinstance(a, str) else (a, b))) for a, b in pairs)
            if top is None or total < top[0]:
                top = (total, pairs)
        for a, b in top[1]:
            i, name = (b, a) if isinstance(a, str) else (a, b)
            best[i] = name
    else:
        for c, i, name in sorted((cost(i, n), i, n) for i in range(len(shapes)) for n in names):
            if i not in best and name not in best.values():
                best[i] = name
    n = 0
    out = []
    for i in range(len(shapes)):
        if i not in best:
            n += 1
        out.append(best.get(i, f'Room {n}'))
    return out


def annotate(out, verts, groups, floor_z, occ, clear, reach, x0, y0, nx, ny, plan):
    def centre(k):
        return x0 + (k % nx + 0.5) * GRID, y0 + (k // nx + 0.5) * GRID

    def cell(x, y):
        cx, cy = int((x - x0) / GRID), int((y - y0) / GRID)
        return cy * nx + cx if 0 <= cx < nx and 0 <= cy < ny else None

    # Rooms: floor split by walls (full height) and shut doors; furniture does not split rooms.
    # Closing 0.3 m gaps seals corners the scan left open (a closet leaked outside through
    # one) without closing doorways, which are at least 0.6 m.
    walls = close_gaps(occupancy(verts, groups, floor_z, x0, y0, nx, ny, kinds=('wall',), band=(0.02, 10.0)),
                       nx, ny, cells=3)
    doors = occupancy(verts, groups, floor_z, x0, y0, nx, ny, kinds=('door',), band=(0.02, 10.0))
    barrier = [w or d for w, d in zip(walls, doors)]
    inside_walls = enclosed(barrier, nx, ny)
    label, regions = components([not b and i for b, i in zip(barrier, inside_walls)], nx, ny)
    keep = [r for r in regions if len(r) * GRID * GRID >= 0.3]      # drop slivers between wall faces
    label = [0] * (nx * ny)
    for i, r in enumerate(keep):
        for k in r:
            label[k] = i + 1
    shapes = room_shapes(keep, barrier, nx)
    names = name_rooms(shapes, plan) if plan else [f'Room {i + 1}' for i in range(len(keep))]

    def room_at(x, y, search=0.3):
        k = cell(x, y)
        if k is not None and label[k]:
            return names[label[k] - 1]
        best = None
        for j in range(nx * ny):
            if label[j]:
                cx, cy = centre(j)
                d = math.hypot(cx - x, cy - y)
                if d <= search and (best is None or d < best[0]):
                    best = (d, names[label[j] - 1])
        return best[1] if best else 'outside'

    door_list = []
    for g in (g for g in groups if g['kind'] == 'door'):
        cx, cy = (g['lo'][0] + g['hi'][0]) / 2, (g['lo'][1] + g['hi'][1]) / 2
        along_x = g['size'][0] >= g['size'][1]
        nxv, nyv = (0.0, 1.0) if along_x else (1.0, 0.0)
        door_list.append({'x': cx, 'y': cy, 'width': max(g['size'][0], g['size'][1]),
                          'yaw': 0.0 if along_x else math.pi / 2,
                          'between': sorted({room_at(cx + 0.25 * nxv, cy + 0.25 * nyv),
                                             room_at(cx - 0.25 * nxv, cy - 0.25 * nyv)})})

    # Narrow passages: the middle line of free space (local maximum of clearance across it)
    # where the space is narrower than NARROW but the robot still fits.
    def ridge(k):
        x, y = k % nx, k // nx
        def peak(a, b):   # not lower than either side, and higher than one: excludes runs along a wall
            return clear[k] >= a and clear[k] >= b and clear[k] > min(a, b)
        across_x = 0 < x < nx - 1 and peak(clear[k - 1], clear[k + 1])
        across_y = 0 < y < ny - 1 and peak(clear[k - nx], clear[k + nx])
        return across_x or across_y
    narrow_ok = [k in reach and clear[k] < NARROW / 2 and ridge(k) for k in range(nx * ny)]
    _, passages = components(narrow_ok, nx, ny, diagonal=True)
    passage_list = []
    for cells in (c for c in passages if len(c) * GRID >= 0.5):
        # median, not minimum: a passage's ends taper to the robot's own width
        k = sorted(cells, key=lambda k: clear[k])[len(cells) // 2]
        px = sum(centre(c)[0] for c in cells) / len(cells)
        py = sum(centre(c)[1] for c in cells) / len(cells)
        passage_list.append({'x': px, 'y': py, 'width': 2 * clear[k], 'length': len(cells) * GRID,
                             'room': room_at(px, py), 'cells': cells})

    # Dock: next to a wall (robot centre 0.33-0.45 m from it) with open floor in front, in a
    # room the resident uses, at least 1 m from a doorway; facing into the room. A suggestion
    # to check on the image.
    service = ('bath', 'closet', 'wc', 'toilet', 'storage', 'cupboard', 'utility')
    dock = None
    for k in reach:
        if not 0.33 <= clear[k] <= 0.45:
            continue
        x, y = k % nx, k // nx
        if not (0 < x < nx - 1 and 0 < y < ny - 1):
            continue
        gx, gy = clear[k + 1] - clear[k - 1], clear[k + nx] - clear[k - nx]
        n = math.hypot(gx, gy)
        if not n:
            continue
        cx, cy = centre(k)
        front = cell(cx + 0.8 * gx / n, cy + 0.8 * gy / n)
        room = names[label[k] - 1] if label[k] else ''
        if front is None or front not in reach or any(w in room.lower() for w in service):
            continue
        if any(math.hypot(cx - d['x'], cy - d['y']) < 1.0 for d in door_list):
            continue    # a parked robot would block the doorway
        if dock is None or clear[front] > dock[0]:
            dock = (clear[front], k, math.atan2(gy, gx), room)

    spots = []
    for i, r in enumerate(keep):
        if any(w in names[i].lower() for w in service):
            continue
        k = max(r, key=lambda k: clear[k])
        spots.append({'room': names[i], 'x': centre(k)[0], 'y': centre(k)[1]})

    with open(os.path.join(out, 'rooms.pgm'), 'wb') as f:
        f.write(f'P5\n{nx} {ny}\n255\n'.encode())
        for y in range(ny - 1, -1, -1):
            f.write(bytes(label[y * nx + x] for x in range(nx)))

    lines = ['# Generated by tools/scan_to_world.py. Coordinates in metres, Gazebo world = map frame.',
             '# rooms.pgm: each cell holds its room id (0 = no room), on the map grid below.',
             f'grid: {{resolution: {GRID}, origin: [{x0:.3f}, {y0:.3f}], width: {nx}, height: {ny}}}',
             'rooms:']
    for i, (name, (area, _)) in enumerate(zip(names, shapes)):
        plan_area = plan.get(name, {}).get('area') if plan else None
        cx = sum(centre(k)[0] for k in keep[i]) / len(keep[i])
        cy = sum(centre(k)[1] for k in keep[i]) / len(keep[i])
        lines.append(f'  - {{id: {i + 1}, name: "{name}", area_m2: {area:.2f}, '
                     f'floorplan_area_m2: {plan_area if plan_area is not None else "null"}, '
                     f'centre: [{cx:.2f}, {cy:.2f}]}}')
    lines.append('doors:')
    for d in door_list:
        lines.append(f'  - {{x: {d["x"]:.2f}, y: {d["y"]:.2f}, yaw: {d["yaw"]:.4f}, width: {d["width"]:.2f}, '
                     f'between: [{", ".join(chr(34) + b + chr(34) for b in d["between"])}]}}')
    lines.append(f'narrow_passages:  # free space narrower than {NARROW} m where the robot still fits')
    for p in passage_list:
        lines.append(f'  - {{x: {p["x"]:.2f}, y: {p["y"]:.2f}, width: {p["width"]:.2f}, '
                     f'length: {p["length"]:.2f}, room: "{p["room"]}"}}')
    if dock:
        dx, dy = centre(dock[1])
        lines.append(f'dock: {{x: {dx:.2f}, y: {dy:.2f}, yaw: {dock[2]:.4f}, room: "{dock[3]}", '
                     'suggested: true}  # check on annotations.png')
    else:
        lines.append('dock: null  # no spot next to a wall with open floor in front: set by hand')
    lines.append('resident_spots:  # most open floor in each room the resident uses')
    for sp in spots:
        lines.append(f'  - {{room: "{sp["room"]}", x: {sp["x"]:.2f}, y: {sp["y"]:.2f}}}')
    with open(os.path.join(out, 'annotations.yaml'), 'w') as f:
        f.write('\n'.join(lines) + '\n')

    palette = [(141, 211, 199), (255, 255, 179), (190, 186, 218), (251, 128, 114), (128, 177, 211),
               (253, 180, 98), (179, 222, 105), (252, 205, 229), (204, 235, 197), (255, 237, 111)]
    narrow_cells = {k for p in passage_list for k in p['cells']}
    marks = {}
    if dock:
        marks.update({j: (220, 30, 30) for j in range(nx * ny)
                      if abs(j % nx - dock[1] % nx) <= 2 and abs(j // nx - dock[1] // nx) <= 2})
    for sp in spots:
        c = cell(sp['x'], sp['y'])
        marks.update({j: (120, 40, 160) for j in range(nx * ny)
                      if abs(j % nx - c % nx) <= 1 and abs(j // nx - c // nx) <= 1})
    rows = []
    for y in range(ny - 1, -1, -1):
        row = []
        for x in range(nx):
            k = y * nx + x
            if k in marks:
                row += marks[k]
            elif doors[k]:
                row += (40, 90, 220)
            elif occ[k] or walls[k]:
                row += (30, 30, 30)
            elif k in narrow_cells:
                row += (230, 120, 20)
            elif label[k]:
                row += palette[(label[k] - 1) % len(palette)]
            else:
                row += (200, 200, 200)
        rows += [[c for px in zip(row[0::3], row[1::3], row[2::3]) for c in px * PREVIEW_SCALE]] * PREVIEW_SCALE
    write_png(os.path.join(out, 'annotations.png'), rows)

    print('Rooms (scan area vs floor plan, colour on annotations.png):')
    colour_names = ['teal', 'pale yellow', 'lilac', 'salmon', 'blue', 'orange', 'green', 'pink', 'mint', 'yellow']
    for i, (name, (area, _)) in enumerate(zip(names, shapes)):
        plan_area = plan.get(name, {}).get('area') if plan else None
        diff = f', floor plan {plan_area:.1f} m2 ({100 * (area - plan_area) / plan_area:+.0f}%)' if plan_area else ''
        print(f'  {name}: {area:.1f} m2{diff} [{colour_names[i % len(colour_names)]}]')
    if plan:
        missing = [n for n in plan if n not in names]
        if missing:
            print('  In the floor plan but not found in the scan: ' + ', '.join(missing))
    print(f'Doors: {len(door_list)}; narrow passages (< {NARROW} m): {len(passage_list)}')
    print('Dock (suggested, red on the image): ' +
          (f'x={centre(dock[1])[0]:.2f} y={centre(dock[1])[1]:.2f} in {dock[3]}' if dock else 'none found'))


MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <static>true</static>
    <link name="house">
      <visual name="visual">
        <geometry><mesh><uri>model://{name}/meshes/visual.obj</uri></mesh></geometry>
      </visual>
      <!-- Collision as a box floor, one low-poly wall mesh and one box per piece of
           furniture: DART ran the AWS house's detailed collision meshes at 0.12x real time. -->
      <collision name="floor">
        <pose>{fx:.3f} {fy:.3f} {fz:.3f} 0 0 0</pose>
        <geometry><box><size>{fsx:.3f} {fsy:.3f} {fsz:.3f}</size></box></geometry>
      </collision>
      <collision name="walls">
        <geometry><mesh><uri>model://{name}/meshes/walls.obj</uri></mesh></geometry>
      </collision>
{furniture}    </link>
  </model>
</sdf>
"""

BOX = """      <collision name="furniture_{i}">
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
      </collision>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>Generated by tools/scan_to_world.py from a Room-mode scan. Private: do not publish.</description>
</model>
"""

# Physics block matches worlds/small_house.world so results are comparable across homes.
WORLD = """<?xml version="1.0"?>
<sdf version="1.10">
  <world name="default">
    <gui>
      <camera name="gzclient_camera">
        <pose>{cx:.2f} {cy:.2f} {cz:.2f} 0 1.2 1.5708</pose>
      </camera>
    </gui>
    <gravity>0 0 -9.8</gravity>
    <physics name="default_physics" default="false" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
      <cast_shadows>false</cast_shadows>
    </light>
    <include>
      <uri>model://{name}</uri>
    </include>
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scan', help='Room-mode OBJ export (.obj or the .zip Polycam gives)')
    ap.add_argument('name', help='home name, e.g. guest_flat (lowercase, no spaces)')
    ap.add_argument('--floorplan', help="the home's RoomPlan floor-plan CSV, to name the rooms")
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'homes'))
    args = ap.parse_args()

    verts, groups = parse(*read_scan(args.scan))
    slab, floor_z = classify(verts, groups)
    out = os.path.join(os.path.abspath(args.out), args.name)
    os.makedirs(os.path.join(out, 'meshes'), exist_ok=True)

    kinds = {}
    for g in groups:
        kinds[g['kind']] = kinds.get(g['kind'], 0) + 1

    write_obj(os.path.join(out, 'meshes', 'visual.obj'), verts,
              [g for g in groups if g['kind'] != 'door'], floor_z, mtl_name='visual.mtl')
    write_obj(os.path.join(out, 'meshes', 'walls.obj'), verts,
              [g for g in groups if g['kind'] == 'wall'], floor_z,
              # The walls' undersides run across the doorways at floor level, and the
              # wheels caught on their edges: TIAGo could not drive through a doorway.
              skip_floor_faces=True)

    boxes, footprints = '', []
    for i, g in enumerate(g for g in groups if g['kind'] == 'furniture'):
        x, y, length, width, yaw = footprint_box([verts[k][:2] for t in g['tris'] for k in t])
        footprints.append((x, y, length, width, yaw))
        boxes += BOX.format(i=i, x=x, y=y, z=(g['lo'][2] + g['hi'][2]) / 2 - floor_z,
                            sx=length, sy=width, sz=g['size'][2], yaw=yaw)
    lo, hi = slab['lo'], slab['hi']
    with open(os.path.join(out, 'model.sdf'), 'w') as f:
        f.write(MODEL_SDF.format(name=args.name, furniture=boxes,
                                 fx=(lo[0] + hi[0]) / 2, fy=(lo[1] + hi[1]) / 2, fz=(lo[2] - hi[2]) / 2,
                                 fsx=slab['size'][0], fsy=slab['size'][1], fsz=slab['size'][2]))
    with open(os.path.join(out, 'model.config'), 'w') as f:
        f.write(MODEL_CONFIG.format(name=args.name))
    with open(os.path.join(out, f'{args.name}.world'), 'w') as f:
        f.write(WORLD.format(name=args.name, cx=(lo[0] + hi[0]) / 2, cy=lo[1] - 2.0,
                             cz=max(slab['size'][0], slab['size'][1]) * 1.2))

    x0, y0 = lo[0] - GRID, lo[1] - GRID
    nx = int((hi[0] - lo[0]) / GRID) + 3
    ny = int((hi[1] - lo[1]) / GRID) + 3
    occ = occupancy(verts, groups, floor_z, x0, y0, nx, ny)
    doors = occupancy(verts, groups, floor_z, x0, y0, nx, ny, kinds=('door',))
    # The floor slab is a rectangle, so it also covers ground outside the walls. Inside is
    # what cannot be reached from the grid's edge with the doors shut.
    border = [k for k in range(nx * ny) if k % nx in (0, nx - 1) or k // nx in (0, ny - 1)]
    outside = flood([not (o or d) for o, d in zip(occ, doors)], border, nx)
    inside = [not o and not out for o, out in zip(occ, outside)]
    if not any(inside):
        print('Warning: the walls do not enclose the home (gaps in the scan?); using the whole slab.')
        inside = [not o for o in occ]
    clear = clearance(occ, nx, ny)
    region = largest_region([i and c > ROBOT_RADIUS for i, c in zip(inside, clear)], nx, ny)
    spawn = max(region, key=lambda k: clear[k])
    sx, sy = x0 + (spawn % nx + 0.5) * GRID, y0 + (spawn // nx + 0.5) * GRID
    free = sum(inside)
    reach = set(region)
    # Floor the robot's body can cover: within one radius of a place its centre can reach.
    to_reach = clearance([k in reach for k in range(nx * ny)], nx, ny)
    covered = sum(1 for k in range(nx * ny) if inside[k] and to_reach[k] <= ROBOT_RADIUS)

    rows = []
    for y in range(ny - 1, -1, -1):          # image top = +y
        row = []
        for x in range(nx):
            k = y * nx + x
            if abs(x - spawn % nx) <= 2 and abs(y - spawn // nx) <= 2:
                row += (220, 30, 30)
            elif occ[k]:
                row += (30, 30, 30)
            elif doors[k]:
                row += (40, 90, 220)
            elif k in reach:
                row += (120, 200, 140)
            elif inside[k]:
                row += (250, 250, 250)
            else:
                row += (200, 200, 200)
        rows += [[c for px in zip(row[0::3], row[1::3], row[2::3]) for c in px * PREVIEW_SCALE]] * PREVIEW_SCALE
    write_png(os.path.join(out, 'preview.png'), rows)

    walls = close_gaps(occupancy(verts, groups, floor_z, x0, y0, nx, ny, band=MAP_BAND), nx, ny)
    # Furniture is drawn as its collision box, which is what the robot bumps into; the
    # laser only sees legs inside it, and points inside an obstacle still suit AMCL.
    for fx, fy, length, width, yaw in footprints:
        c, s_ = math.cos(yaw), math.sin(yaw)
        for k in range(nx * ny):
            px, py = x0 + (k % nx + 0.5) * GRID - fx, y0 + (k // nx + 0.5) * GRID - fy
            if abs(px * c + py * s_) <= length / 2 and abs(-px * s_ + py * c) <= width / 2:
                walls[k] = 1
    with open(os.path.join(out, 'map.pgm'), 'wb') as f:
        f.write(f'P5\n{nx} {ny}\n255\n'.encode())
        for y in range(ny - 1, -1, -1):
            f.write(bytes(0 if walls[y * nx + x] else 205 if outside[y * nx + x] else 254
                          for x in range(nx)))
    with open(os.path.join(out, 'map.yaml'), 'w') as f:
        f.write(f'image: map.pgm\nmode: trinary\nresolution: {GRID}\norigin: [{x0:.3f}, {y0:.3f}, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

    annotate(out, verts, groups, floor_z, occ, clear, reach, x0, y0, nx, ny,
             read_floorplan(args.floorplan) if args.floorplan else {})

    print(f'Wrote {out}')
    print('Elements: ' + ', '.join(f'{v} {k}' for k, v in sorted(kinds.items())))
    print(f'Doors removed: {kinds.get("door", 0)}; furniture collision boxes: {kinds.get("furniture", 0)}')
    print(f'Free floor inside: {free * GRID * GRID:.1f} m2. TIAGo (radius {ROBOT_RADIUS} m) can cover '
          f'{100 * covered / max(free, 1):.0f}% of it; its centre can reach {100 * len(region) / max(free, 1):.0f}%.')
    print(f'Spawn (most open spot): x={sx:.2f} y={sy:.2f}, clearance {clear[spawn]:.2f} m')
    pose = f'x:={sx:.2f} y:={sy:.2f} yaw:=0'
    print('Launch with:')
    print(f'  ros2 launch elderly_care_robot_sim robot_in_house.launch.py '
          f'world:=../homes/{args.name}/{args.name}.world {pose}')
    print('Then Nav2 in a second terminal:')
    print(f'  ros2 launch elderly_care_robot_sim navigation.launch.py world:=../homes/{args.name}/{args.name}.world '
          f'map:=$(ros2 pkg prefix elderly_care_robot_sim)/share/elderly_care_robot_sim/homes/{args.name}/map.yaml {pose}')


if __name__ == '__main__':
    sys.exit(main())
