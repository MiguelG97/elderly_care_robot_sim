#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "matplotlib", "pyyaml", "mcap", "mcap-ros2-support"]
# ///
"""
report: turn task_runner runs into tables, figures and one HTML page. Runs on the Mac, no ROS.

  uv run tools/report.py ../runs --batch pilot     one batch
  uv run tools/report.py ../runs --paper           the batches chosen in paper/paper.yaml

Reads <runs>/batches/<batch>/: batch.yaml and every runs/*/ folder that has events.csv and
run.yaml (copied from the instance with rsync), plus its bag (bag/*.mcap: /ground_truth/odom
and /people/odom) and the home's map.pgm/annotations.yaml from --homes.

--batch writes to --out (default <runs>/batches/<batch>/report).
--paper writes each question to <runs>/paper/<question>/<resident>/, without floor plans.
paper.yaml lists the batches behind each question:
  questions:
    Q1: [q1_empty]
    Q3_Q4: []
    Q8: [main, main_more_seeds]
    Q9: [q9_static]
Batches with the same resident type are pooled; different types are kept apart. Two runs
with the same home, mode, seed and resident type are refused, since they would count twice.

Each output holds:
  tables.csv, tables.tex   dock returns (Q8), patrol by room (Q1), closeness to the resident
                           and encounter direction (Q3, Q4), failures with the resident's
                           position (Q9)
  *.png                    one figure per table, and a floor plan per home with the robot's
                           paths, failures and close encounters
  report.html              all of it on one page, images embedded

The floor plans show a real home's layout: keep report.html private. Distances are
centre to centre from ground truth (A3), so 0.275 m of it is the robot's own radius.
"""
import argparse
import base64
import csv
import glob
import io
import math
import os
import shutil
import statistics
import sys
import tempfile
from collections import Counter, defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from mcap.records import Channel, Message, Schema  # noqa: E402
from mcap.stream_reader import StreamReader  # noqa: E402
from mcap_ros2.decoder import DecoderFactory  # noqa: E402

# Hall's personal-distance limit (A19) and the comfort limit used since Phase A.
NEAR, CLOSE = 1.2, 0.8
# Robot within 60 degrees of where the resident faces = in view, beyond 120 = from behind.
FRONT, BEHIND = math.radians(60), math.radians(120)
MODES = ('baseline', 'social')
COLOURS = {'baseline': '#4C78A8', 'social': '#E45756'}

DESIGN = """
<h2>How the runs were designed</h2>
<ul>
<li><b>Home and robot:</b> a scanned home turned into a simulation, with a TIAGo base (0.55 m
across) that starts every run on its charging dock.</li>
<li><b>Job:</b> the robot patrols one goal in each room it can reach. At random moments
(60–240 s apart) it is called back to the dock, wherever it is; 3 calls per run.</li>
<li><b>Resident:</b> {resident}</li>
<li><b>Two navigation modes</b>, everything else identical:
  <ul><li><b>baseline</b>: standard navigation (Nav2). The resident is just an obstacle its laser
  sees.</li>
  <li><b>social</b>: the same, plus a personal-space zone around the resident (larger in front of
  them) that the robot prefers not to enter.</li></ul></li>
<li><b>Seeds:</b> each seed fixes the call times, so both modes face the same schedule.</li>
<li><b>Measurement:</b> exact positions from the simulator, not the robot's own estimate.</li>
</ul>"""

RESIDENTS = {
    'scripted': "one person walking a scripted daily routine (bedroom, bathroom, day rooms) at an "
                "older adult's pace. They do not react to the robot yet.",
    'static': 'one person standing still at a chosen spot.',
    'none': 'none, the home is empty.',
}
# Report sections paper.yaml can name, as the id each table and figure title starts with.
PAPER_SECTIONS = {'Q1': 'Q1', 'Q3_Q4': 'Q3, Q4', 'Q8': 'Q8', 'Q9': 'Q9'}

# Question, why it matters, and what each column means; keyed by the table's id.
QUESTIONS = {
    'Q8': {
        'question': 'Can the robot return to its dock at any time?',
        'why': 'A care robot must get back to its charger whenever called, whatever it is doing and '
               'wherever the resident is. If the layout or the resident stops it, it runs flat.',
        'columns': {
            'Runs': 'runs in this mode',
            'Calls': 'times the robot was called back to the dock (3 per run)',
            'Docked': 'calls where it reached the dock: navigation reported success and its true '
                      'position was within 0.25 m and 0.25 rad of the dock',
            'Rate': 'Docked ÷ Calls',
            'Median / Max time (s)': 'from the call until docked, in simulated seconds',
            'Median / Max error (m)': 'how far from the exact dock spot it stopped',
            'Median Nav2 estimate error (m)': 'how far the robot\'s own idea of its position was from '
                                              'the truth; tells a lost robot apart from a blocked one',
            'Failures': 'why a call failed: timeout, stopped off the dock, or navigation gave up',
        },
    },
    'Q1': {
        'question': 'Can the robot reach all areas?',
        'why': 'A care robot is only useful in the rooms it can get into. A room it cannot enter is a '
               'room where it cannot help, and points to a layout that may need changing.',
        'columns': {
            'Room': 'the room holding the patrol goal (closets are not patrolled)',
            'Goals': 'times the robot set off for this room',
            'Reached': 'times it arrived',
            'Failed': 'times it tried and could not arrive',
            'Interrupted by dock call': 'a dock call came first, so the attempt says nothing either way',
            'Failure reasons': 'nav2_aborted 208: navigation found no path. nav2_aborted: navigation '
                               'gave up. stopped_short: it reported arrival but was too far from the goal',
        },
    },
    'Q3, Q4': {'question': 'Does the robot come up on the resident from behind, or face to face?'},
    'Q9': {'question': 'Does access depend on where the resident is?'},
}


def takeaway_q8(runs):
    parts = []
    for mode in MODES:
        dock = [e for r in runs if r['mode'] == mode for e in r['events'] if e['kind'] == 'dock']
        ok = [e for e in dock if e['result'] == 'success']
        if dock:
            parts.append(f"{mode} {len(ok)}/{len(dock)} docked, median {med([float(e['duration_s']) for e in ok])} s")
    worst = max((float(e['err_xy_m']) for r in runs for e in r['events']
                 if e['kind'] == 'dock' and e['result'] == 'success'), default=None)
    end = f'; every docked return ended within {worst:.3f} m of the dock (pass mark 0.25 m)' if worst else ''
    n = len({(r['home'], r['seed']) for r in runs})
    return (f"{'; '.join(parts)}{end}. Based on {n} home–seed pairs per mode: enough to show the "
            "robot can dock, not yet to tell the modes apart.")


def takeaway_q1(runs, homes_dir):
    parts = []
    for home in sorted({r['home'] for r in runs}):
        es = [e for r in runs if r['home'] == home for e in r['events'] if e['kind'] == 'patrol']
        rooms = []
        for room in sorted({e['target'] for e in es}):
            c = Counter(e['result'] for e in es if e['target'] == room)
            rooms.append(f"{room} {c['success']} of {sum(c.values())} ({c['fail']} failed, "
                         f"{c['interrupted']} interrupted)")
        text = f"patrol reached {', '.join(rooms)}"
        ann = os.path.join(homes_dir, home, 'annotations.yaml')
        if os.path.isfile(ann):
            a = yaml.safe_load(open(ann))
            blocked = a.get('unreachable_rooms') or []
            text = (f"on the map, {len(a['rooms']) - len(blocked)} of {len(a['rooms'])} rooms have a spot "
                    f"the robot fits; {text}")
        parts.append(text)
    caveat = '' if all(r['resident'] == 'none' for r in runs) else (
        ' A resident was present during these runs, so a room missed here may be blocked by them '
        'and not by the layout. Q1 needs a run with an empty home.')
    return f"{'; '.join(parts)}.{caveat}"


def read_bag(path):
    """(robot, resident) arrays of rows t, x, y, yaw from an mcap bag, sim time from headers."""
    schemas, channels, decoders = {}, {}, DecoderFactory()
    out = {'/ground_truth/odom': [], '/people/odom': []}
    records = StreamReader(open(path, 'rb'), skip_magic=False).records
    while True:
        try:
            rec = next(records)
        except StopIteration:
            break
        except Exception:
            # The batch stops the recorder with the rest of the run, so the file's index at
            # the end is often cut off; every message before it is intact.
            break
        if isinstance(rec, Schema):
            schemas[rec.id] = rec
        elif isinstance(rec, Channel):
            channels[rec.id] = rec
        elif isinstance(rec, Message):
            ch = channels[rec.channel_id]
            if ch.topic not in out:
                continue
            m = decoders.decoder_for(ch.message_encoding, schemas[ch.schema_id])(rec.data)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            out[ch.topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, yaw))
    robot, resident = (np.array(sorted(out[k])) for k in ('/ground_truth/odom', '/people/odom'))
    return robot, resident


def encounters(robot, resident):
    """Closeness metrics on the robot's timeline, with the resident interpolated onto it."""
    if len(robot) < 2 or len(resident) < 2:
        return None
    t = robot[:, 0]
    keep = (t >= resident[0, 0]) & (t <= resident[-1, 0])
    t, rx, ry = t[keep], robot[keep, 1], robot[keep, 2]
    px, py = np.interp(t, resident[:, 0], resident[:, 1]), np.interp(t, resident[:, 0], resident[:, 2])
    # yaw is circular: interpolate its sine and cosine
    pyaw = np.arctan2(np.interp(t, resident[:, 0], np.sin(resident[:, 3])),
                      np.interp(t, resident[:, 0], np.cos(resident[:, 3])))
    d = np.hypot(rx - px, ry - py)
    dt = np.diff(t, append=t[-1])
    near = d < NEAR
    found = []
    i = 0
    while i < len(d):
        if not near[i]:
            i += 1
            continue
        j = i
        while j < len(d) and near[j]:
            j += 1
        k = i + int(np.argmin(d[i:j]))
        rel = abs(math.remainder(math.atan2(ry[k] - py[k], rx[k] - px[k]) - pyaw[k], math.tau))
        side = 'front' if rel <= FRONT else 'behind' if rel >= BEHIND else 'side'
        found.append({'t': t[k], 'min_m': float(d[k]), 'dur_s': float(t[j - 1] - t[i]), 'side': side,
                      'x': float(rx[k]), 'y': float(ry[k])})
        i = j
    return {'min_m': float(d.min()), 'near_s': float(dt[near].sum()), 'close_s': float(dt[d < CLOSE].sum()),
            'encounters': found}


def read_pgm(path):
    with open(path, 'rb') as f:
        raw = f.read()
    header, pos = [], 0
    while len(header) < 4:
        end = raw.index(b'\n', pos)
        header += raw[pos:end].split()
        pos = end + 1
    w, h = int(header[1]), int(header[2])
    return np.frombuffer(raw[pos:pos + w * h], dtype=np.uint8).reshape(h, w)


def load(runs_dir, batch):
    root = os.path.join(runs_dir, 'batches', batch)
    record = os.path.join(root, 'batch.yaml')
    if not os.path.isfile(record):
        sys.exit(f'No batch record: {record}')
    resident = yaml.safe_load(open(record))['resident']
    runs = []
    for d in sorted(glob.glob(os.path.join(root, 'runs', '*'))):
        ev, rec = os.path.join(d, 'events.csv'), os.path.join(d, 'run.yaml')
        if not (os.path.isfile(ev) and os.path.isfile(rec)):
            continue
        meta = yaml.safe_load(open(rec))
        p = meta['parameters']
        run = {'id': meta['run_id'], 'batch': batch, 'resident': resident, 'home': p['home'], 'seed': p['seed'],
               'mode': 'social' if str(p['social']).lower() == 'true' else 'baseline',
               'events': list(csv.DictReader(open(ev))), 'dock': meta.get('dock'), 'enc': None, 'path': None}
        bags = glob.glob(os.path.join(d, 'bag', '*.mcap'))
        if bags:
            robot, resident = read_bag(bags[0])
            run['enc'] = encounters(robot, resident)
            if len(robot):
                run['path'] = robot[:, 1:3]
        else:
            print(f'{run["id"]}: no bag, closeness metrics skipped', file=sys.stderr)
        runs.append(run)
    return runs


def med(xs):
    return round(statistics.median(xs), 2) if xs else ''


def tables(runs):
    t = {}
    rows = []
    for home in sorted({r['home'] for r in runs}):
        for mode in MODES:
            rs = [r for r in runs if r['home'] == home and r['mode'] == mode]
            dock = [e for r in rs for e in r['events'] if e['kind'] == 'dock']
            ok = [e for e in dock if e['result'] == 'success']
            if not dock:
                continue
            fails = Counter(e['reason'] for e in dock if e['result'] != 'success')
            rows.append([home, mode, len(rs), len(dock), len(ok), f'{100 * len(ok) / len(dock):.0f}%',
                         med([float(e['duration_s']) for e in ok]),
                         max((float(e['duration_s']) for e in ok), default=''),
                         med([float(e['err_xy_m']) for e in ok]),
                         max((float(e['err_xy_m']) for e in ok), default=''),
                         med([float(e['loc_err_m']) for e in dock if e['loc_err_m']]),
                         ', '.join(f'{k} ×{v}' for k, v in fails.items()) or '—'])
    t['Q8 · Return to dock'] = (['Home', 'Mode', 'Runs', 'Calls', 'Docked', 'Rate', 'Median time (s)',
                                 'Max time (s)', 'Median error (m)', 'Max error (m)',
                                 'Median Nav2 estimate error (m)', 'Failures'], rows)

    rows = []
    for home in sorted({r['home'] for r in runs}):
        rooms = sorted({e['target'] for r in runs if r['home'] == home for e in r['events'] if e['kind'] == 'patrol'})
        for room in rooms:
            for mode in MODES:
                es = [e for r in runs if r['home'] == home and r['mode'] == mode
                      for e in r['events'] if e['kind'] == 'patrol' and e['target'] == room]
                if not es:
                    continue
                c = Counter(e['result'] for e in es)
                why = Counter(f"{e['reason']}{' ' + e['nav2_error'] if e['nav2_error'] else ''}"
                              for e in es if e['result'] == 'fail')
                rows.append([home, room, mode, len(es), c['success'], c['fail'], c['interrupted'],
                             ', '.join(f'{k} ×{v}' for k, v in why.items()) or '—'])
    t['Q1 · Patrol goals by room'] = (['Home', 'Room', 'Mode', 'Goals', 'Reached', 'Failed',
                                       'Interrupted by dock call', 'Failure reasons'], rows)

    rows = []
    for home in sorted({r['home'] for r in runs}):
        for mode in MODES:
            rs = [r for r in runs if r['home'] == home and r['mode'] == mode and r['enc']]
            if not rs:
                continue
            enc = [e for r in rs for e in r['enc']['encounters']]
            side = Counter(e['side'] for e in enc)
            rows.append([home, mode, len(rs), round(min(r['enc']['min_m'] for r in rs), 2),
                         med([r['enc']['min_m'] for r in rs]),
                         round(statistics.mean(r['enc']['near_s'] for r in rs), 1),
                         round(statistics.mean(r['enc']['close_s'] for r in rs), 1),
                         len(enc), side['front'], side['side'], side['behind']])
    t['Q3, Q4 · Closeness to the resident'] = (
        ['Home', 'Mode', 'Runs', 'Closest (m)', 'Median closest per run (m)',
         f'Time < {NEAR} m per run (s)', f'Time < {CLOSE} m per run (s)', f'Encounters < {NEAR} m',
         'In front', 'Side', 'From behind'], rows)

    rows = []
    for r in runs:
        for e in r['events']:
            if e['result'] == 'fail':
                rows.append([r['home'], r['mode'], r['seed'], e['kind'], e['target'],
                             f"{e['reason']}{' ' + e['nav2_error'] if e['nav2_error'] else ''}",
                             e['robot_room'], e['resident_room'] or '—', e['resident_doorway'] or '—',
                             e['resident_dist_m']])
    t['Q9 · Failures and where the resident was'] = (
        ['Home', 'Mode', 'Seed', 'Job', 'Target', 'Reason', 'Robot room (start)',
         'Resident room (start)', 'Resident doorway (start)', 'Resident distance (m, start)'], rows)
    if all(r['resident'] == 'none' for r in runs):
        del t['Q3, Q4 · Closeness to the resident'], t['Q9 · Failures and where the resident was']
    if not t['Q8 · Return to dock'][1]:
        del t['Q8 · Return to dock']
    return t


def fig_png(fig, out, name):
    path = os.path.join(out, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return path


def figures(runs, homes_dir, out):
    figs = []
    homes = sorted({r['home'] for r in runs})

    fig, axes = plt.subplots(1, 2, figsize=(9, 3.4))
    for i, mode in enumerate(MODES):
        dock = [e for r in runs if r['mode'] == mode for e in r['events'] if e['kind'] == 'dock']
        ok = [float(e['duration_s']) for e in dock if e['result'] == 'success']
        axes[0].bar(i, 100 * len(ok) / max(len(dock), 1), color=COLOURS[mode])
        axes[0].text(i, 2, f'{len(ok)}/{len(dock)}', ha='center', color='white')
        axes[1].scatter(np.full(len(ok), i) + np.random.default_rng(0).uniform(-0.12, 0.12, len(ok)), ok,
                        color=COLOURS[mode], alpha=0.8)
    for ax, label in zip(axes, ('Docked (% of calls)', 'Time to dock (s)')):
        ax.set_xticks([0, 1], MODES)
        ax.set_ylabel(label)
        ax.spines[['top', 'right']].set_visible(False)
    axes[0].set_ylim(0, 105)
    fig.suptitle('Q8 · Return to dock')
    figs.append(('Q8 · Return to dock', fig_png(fig, out, 'q8_dock.png')))

    rooms = sorted({e['target'] for r in runs for e in r['events'] if e['kind'] == 'patrol'})
    fig, ax = plt.subplots(figsize=(9, 3.4))
    w = 0.38
    for i, mode in enumerate(MODES):
        for j, room in enumerate(rooms):
            es = [e for r in runs if r['mode'] == mode for e in r['events']
                  if e['kind'] == 'patrol' and e['target'] == room]
            c = Counter(e['result'] for e in es)
            x, bottom = j + (i - 0.5) * w, 0
            for res, colour in (('success', '#54A24B'), ('fail', '#E45756'), ('interrupted', '#BAB0AC')):
                ax.bar(x, c[res], w * 0.95, bottom=bottom, color=colour,
                       label=res if (i, j) == (0, 0) else None)
                bottom += c[res]
            ax.text(x, bottom + 0.2, mode[0].upper(), ha='center', fontsize=8)
    ax.set_xticks(range(len(rooms)), rooms)
    ax.set_ylabel('Patrol goals')
    ax.legend(frameon=False, ncols=3, loc='upper left')
    ax.spines[['top', 'right']].set_visible(False)
    ax.set_title('Q1 · Patrol goals by room (B = baseline, S = social)')
    figs.append(('Q1 · Patrol goals by room', fig_png(fig, out, 'q1_patrol.png')))

    if any(r['enc'] for r in runs):
        fig, axes = plt.subplots(1, 2, figsize=(9, 3.4))
        bins = np.arange(0, 3.01, 0.1)
        for mode in MODES:
            enc = [e for r in runs if r['mode'] == mode and r['enc'] for e in r['enc']['encounters']]
            axes[0].hist([e['min_m'] for e in enc], bins=bins, alpha=0.6, color=COLOURS[mode], label=mode)
        axes[0].axvline(CLOSE, color='k', ls=':', lw=1)
        axes[0].set_xlim(0, NEAR)
        axes[0].set_xlabel('Closest distance in each encounter (m)')
        axes[0].set_ylabel('Encounters')
        axes[0].legend(frameon=False)
        sides = ('front', 'side', 'behind')
        for i, mode in enumerate(MODES):
            c = Counter(e['side'] for r in runs if r['mode'] == mode and r['enc'] for e in r['enc']['encounters'])
            axes[1].bar(np.arange(3) + (i - 0.5) * w, [c[s] for s in sides], w * 0.95, color=COLOURS[mode], label=mode)
        axes[1].set_xticks(range(3), ['in front', 'side', 'from behind'])
        axes[1].set_ylabel('Encounters')
        for ax in axes:
            ax.spines[['top', 'right']].set_visible(False)
        fig.suptitle(f'Q3, Q4 · Encounters within {NEAR} m of the resident')
        figs.append(('Q3, Q4 · Encounters', fig_png(fig, out, 'q3q4_encounters.png')))

    for home in homes:
        pgm = os.path.join(homes_dir, home, 'map.pgm')
        meta = os.path.join(homes_dir, home, 'map.yaml')
        if not os.path.isfile(pgm):
            print(f'{home}: no map in {homes_dir}, floor plan skipped', file=sys.stderr)
            continue
        m = yaml.safe_load(open(meta))
        img = read_pgm(pgm)
        res, (ox, oy) = m['resolution'], m['origin'][:2]
        extent = (ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res)
        fig, axes = plt.subplots(1, 2, figsize=(11, 5.5))
        for ax, mode in zip(axes, MODES):
            ax.imshow(img, cmap='gray', extent=extent, vmin=0, vmax=255)
            rs = [r for r in runs if r['home'] == home and r['mode'] == mode]
            for r in rs:
                if r['path'] is not None:
                    ax.plot(*r['path'].T, color=COLOURS[mode], lw=0.6, alpha=0.35)
                for e in r['events']:
                    if e['result'] == 'fail' and e['end_x']:
                        ax.plot(float(e['end_x']), float(e['end_y']), 'x', color='#B279A2', ms=9, mew=2)
                    if e['result'] == 'fail' and e['goal_x']:
                        ax.plot(float(e['goal_x']), float(e['goal_y']), 'o', mfc='none', mec='#B279A2', ms=9)
                for e in (r['enc']['encounters'] if r['enc'] else []):
                    if e['min_m'] < CLOSE:
                        ax.plot(e['x'], e['y'], 'o', color='#F58518', ms=5)
            d = rs[0]['dock'] if rs else None
            if d:
                ax.plot(d['x'], d['y'], 's', color='#54A24B', ms=9)
            ax.set_title(f'{home} · {mode} ({len(rs)} runs)')
            ax.set_xticks([])
            ax.set_yticks([])
        handles = [plt.Line2D([], [], color='#54A24B', marker='s', ls='', label='dock'),
                   plt.Line2D([], [], color='#B279A2', marker='o', mfc='none', ls='', label='failed goal'),
                   plt.Line2D([], [], color='#B279A2', marker='x', ls='', label='where it ended'),
                   plt.Line2D([], [], color='#F58518', marker='o', ls='', label=f'robot < {CLOSE} m from resident')]
        fig.legend(handles=handles, loc='lower center', ncols=4, frameon=False)
        figs.append((f'Floor plan · {home}', fig_png(fig, out, f'plan_{home}.png')))
    return figs


def write_tables(t, out):
    with open(os.path.join(out, 'tables.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        for name, (head, rows) in t.items():
            w.writerow([name])
            w.writerow(head)
            w.writerows(rows)
            w.writerow([])
    tex = []
    for name, (head, rows) in t.items():
        esc = lambda s: str(s).replace('%', r'\%').replace('_', r'\_').replace('×', r'$\times$')
        tex += [r'\begin{table}[t]', r'\centering', r'\small', rf'\caption{{{esc(name)}}}',
                r'\begin{tabular}{' + 'l' * len(head) + '}', r'\hline',
                ' & '.join(esc(h) for h in head) + r' \\', r'\hline']
        tex += [' & '.join(esc(c) for c in row) + r' \\' for row in rows]
        tex += [r'\hline', r'\end{tabular}', r'\end{table}', '']
    open(os.path.join(out, 'tables.tex'), 'w').write('\n'.join(tex))


def write_html(t, figs, runs, out, batch, homes_dir):
    def img(path):
        return 'data:image/png;base64,' + base64.b64encode(open(path, 'rb').read()).decode()
    def table(head, rows):
        h = ''.join(f'<th>{c}</th>' for c in head)
        b = ''.join('<tr>' + ''.join(f'<td>{c}</td>' for c in row) + '</tr>' for row in rows)
        return f'<table><thead><tr>{h}</tr></thead><tbody>{b or "<tr><td>none</td></tr>"}</tbody></table>'
    fig_for = dict(figs)
    sections = []
    for name, (head, rows) in t.items():
        qid = name.split(' · ')[0]
        key = next((k for k in fig_for if k.split(' · ')[0] == qid), None)
        pic = f'<img src="{img(fig_for[key])}" alt="{name}">' if key else ''
        info = QUESTIONS.get(qid, {})
        intro = ''
        if info.get('question'):
            intro += f'<p class="q">{info["question"]}</p>'
        if info.get('why'):
            intro += f'<p><b>Why it matters:</b> {info["why"]}</p>'
        if qid == 'Q8':
            intro += f'<p class="take"><b>Result:</b> {takeaway_q8(runs)}</p>'
        if qid == 'Q1':
            intro += f'<p class="take"><b>Result:</b> {takeaway_q1(runs, homes_dir)}</p>'
        cols = ''
        if info.get('columns'):
            cols = '<details><summary>What each column means</summary><dl>' + ''.join(
                f'<dt>{k}</dt><dd>{v}</dd>' for k, v in info['columns'].items()) + '</dl></details>'
        sections.append(f'<h2>{name}</h2>{intro}{pic}{table(head, rows)}{cols}')
    for name, path in figs:
        if name.startswith('Floor plan'):
            sections.append(f'<h2>{name}</h2><p class="note">Private: shows the layout of a real home.</p>'
                            f'<img src="{img(path)}" alt="{name}">')
    homes = ', '.join(sorted({r['home'] for r in runs}))
    seeds = ', '.join(str(s) for s in sorted({r['seed'] for r in runs}))
    html = f"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Run results · {batch}</title>
<style>
:root{{--bg:#fff;--fg:#1d1d1f;--muted:#6e6e73;--line:#e5e5ea}}
@media (prefers-color-scheme:dark){{:root{{--bg:#1c1c1e;--fg:#f2f2f7;--muted:#a1a1a6;--line:#3a3a3c}}}}
body{{background:var(--bg);color:var(--fg);font:15px/1.5 -apple-system,system-ui,sans-serif;max-width:1000px;margin:0 auto;padding:24px 16px}}
h1{{font-size:24px;margin:0 0 4px}} h2{{font-size:18px;margin:36px 0 8px}}
.note,.meta{{color:var(--muted)}} img{{max-width:100%;background:#fff;border-radius:6px}}
table{{border-collapse:collapse;width:100%;font-size:13px;margin-top:8px;display:block;overflow-x:auto}}
th,td{{border-bottom:1px solid var(--line);padding:6px 8px;text-align:left;white-space:nowrap}}
th{{color:var(--muted);font-weight:600}}
.q{{font-size:17px;font-style:italic;margin:0 0 8px}} .take{{border-left:3px solid #54A24B;padding-left:10px}}
dt{{font-weight:600;margin-top:6px}} dd{{margin:0 0 0 12px;color:var(--muted)}} details{{margin-top:10px}}
</style></head><body>
<h1>Run results · {batch}</h1>
<p class="meta">{len(runs)} runs · homes: {homes} · seeds: {seeds} · resident: {runs[0]['resident']} · baseline vs social navigation.
Positions from simulation ground truth; distances centre to centre (robot radius 0.275 m).
Resident and robot rooms are logged at the start of each job.</p>
{DESIGN.format(resident=RESIDENTS.get(runs[0]["resident"], runs[0]["resident"]))}
{''.join(sections)}
</body></html>"""
    path = os.path.join(out, 'report.html')
    open(path, 'w').write(html)
    return path


def build_paper(runs_dir, homes_dir):
    base = os.path.join(runs_dir, 'paper')
    spec = os.path.join(base, 'paper.yaml')
    if not os.path.isfile(spec):
        sys.exit(f'No {spec}')
    questions = yaml.safe_load(open(spec)).get('questions') or {}
    unknown = set(questions) - set(PAPER_SECTIONS)
    if unknown:
        sys.exit(f"Unknown in paper.yaml: {', '.join(sorted(unknown))}. Use {', '.join(PAPER_SECTIONS)}")
    loaded, plan = {}, []
    for q, batches in questions.items():
        runs = []
        for b in batches or []:
            if b not in loaded:
                loaded[b] = load(runs_dir, b)
            if not loaded[b]:
                sys.exit(f'{q}: batch {b} has no finished runs')
            runs += loaded[b]
        groups = defaultdict(list)
        for r in runs:
            groups[r['resident']].append(r)
        for resident, rs in groups.items():
            twice = [k for k, n in Counter((r['home'], r['mode'], r['seed']) for r in rs).items() if n > 1]
            if twice:
                sys.exit(f'{q}, resident {resident}: same run in two batches: {twice}')
        plan.append((q, batches or [], groups))
    for q, batches, groups in plan:
        qdir = os.path.join(base, q)
        shutil.rmtree(qdir, ignore_errors=True)
        if not groups:
            print(f'{q}: no batches chosen')
            continue
        section = PAPER_SECTIONS[q]
        for resident, rs in groups.items():
            t = {k: v for k, v in tables(rs).items() if k.split(' · ')[0] == section}
            if not t:
                print(f'{q}, resident {resident}: these runs do not measure it')
                continue
            out = os.path.join(qdir, resident)
            os.makedirs(out)
            write_tables(t, out)
            figs = []
            with tempfile.TemporaryDirectory() as tmp:
                for name, path in figures(rs, homes_dir, tmp):
                    if name.split(' · ')[0] == section:
                        kept = shutil.copy(path, out)
                        figs.append((name, kept))
            used = sorted({r['batch'] for r in rs})
            print(write_html(t, figs, rs, out, f"{q} · {resident} ({', '.join(used)})", homes_dir))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', help='folder holding the run folders (copied from the instance)')
    ap.add_argument('--batch', help='batch name given to run_batch.sh, e.g. pilot')
    ap.add_argument('--paper', action='store_true', help='build the question outputs from paper/paper.yaml')
    ap.add_argument('--homes', default=os.path.join(here, '..', 'homes'), help='folder holding <home>/map.pgm')
    ap.add_argument('--out', help='output folder (default <runs>/report/<batch>)')
    a = ap.parse_args()
    if bool(a.batch) == a.paper:
        ap.error('give either --batch <name> or --paper')
    if a.paper:
        build_paper(a.runs, a.homes)
        return
    out = a.out or os.path.join(a.runs, 'batches', a.batch, 'report')
    os.makedirs(out, exist_ok=True)
    runs = load(a.runs, a.batch)
    if not runs:
        sys.exit(f'No runs with events.csv and run.yaml in batch {a.batch}')
    t = tables(runs)
    write_tables(t, out)
    figs = figures(runs, a.homes, out)
    print(write_html(t, figs, runs, out, a.batch, a.homes))


if __name__ == '__main__':
    main()
