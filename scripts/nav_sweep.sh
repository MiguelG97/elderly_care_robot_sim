#!/usr/bin/env bash
# nav_sweep: send the robot to one room with several controller settings and compare them.
#
# Usage (same place as run_batch.sh, from a sourced workspace):
#   scripts/nav_sweep.sh <matrix file> <name>
#
# Each variant is a patched copy of config/nav2_params.yaml, run as its own batch
# <name>_<variant> with no resident and no bag. Settings: ROOM [Bathroom], RESULTS [~/runs].
#   control  the current settings
#   cost4    obstacle cost falls off more slowly, so a doorway's centre costs more
#   noshim   MPPI alone, without the rotation shim
#   xy10, xy05  Nav2 counts a goal reached within 0.10 / 0.05 m of it (instead of 0.15)
# RETURNS=1 adds a dock call during the trip, to test docking too.
# ONLY="<variants>" runs just those. Ends with a table per variant: room trips and arrivals,
# dock calls and dockings, median docking time and distance left to the dock.

set -u

MATRIX=${1:?usage: nav_sweep.sh <matrix file> <name>}
NAME=${2:?usage: nav_sweep.sh <matrix file> <name>}
HERE=$(cd "$(dirname "$0")" && pwd)
export RESULTS=${RESULTS:-$HOME/runs}
export ROOM=${ROOM:-Bathroom} RESIDENT=none TASK=room RETURNS=${RETURNS:-0} BAG=0
OUT=$RESULTS/nav_variants/$NAME
mkdir -p "$OUT"

VARIANTS=(control cost4 noshim xy10 xy05)
if [ -n "${ONLY:-}" ]; then
  read -r -a VARIANTS <<< "$ONLY"
fi

python3 - "$HERE/../config/nav2_params.yaml" "$OUT" <<'PY' || exit 1
import copy, os, sys, yaml
base = yaml.safe_load(open(sys.argv[1]))

def cost4(p):
    for name in ('local_costmap', 'global_costmap'):
        p[name][name]['ros__parameters']['inflation_layer']['cost_scaling_factor'] = 4.0

def noshim(p):
    c = p['controller_server']['ros__parameters']['FollowPath']
    c['plugin'] = c.pop('primary_controller')
    for key in ('angular_dist_threshold', 'forward_sampling_distance', 'rotate_to_heading_angular_vel',
                'max_angular_accel', 'simulate_ahead_time'):
        del c[key]

def xy(tol):
    def change(p):
        p['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance'] = tol
    return change

for name, changes in {'control': [], 'cost4': [cost4], 'noshim': [noshim],
                      'xy10': [xy(0.10)], 'xy05': [xy(0.05)]}.items():
    p = copy.deepcopy(base)
    for change in changes:
        change(p)
    yaml.safe_dump(p, open(os.path.join(sys.argv[2], name + '.yaml'), 'w'), sort_keys=False)
PY

for V in "${VARIANTS[@]}"; do
  echo "=== $V"
  # Finished runs are skipped, so the second pass only retries runs lost to a failed startup.
  for _ in 1 2; do
    PURPOSE="Controller sweep to $ROOM, variant $V" NAV_PARAMS=$OUT/$V.yaml \
      "$HERE/run_batch.sh" "$MATRIX" "${NAME}_$V" || exit $?
  done
done

python3 - "$RESULTS/batches" "$NAME" "${VARIANTS[@]}" <<'PY'
import csv, glob, os, statistics, sys
root, name, variants = sys.argv[1], sys.argv[2], sys.argv[3:]
print(f"\n{'variant':16}{'rooms':>7}{'reached':>9}{'docks':>7}{'docked':>8}{'dock s':>8}{'dock left m':>13}")
for v in variants:
    rows = []
    for f in glob.glob(os.path.join(root, f'{name}_{v}', 'runs', '*', 'events.csv')):
        if os.path.isfile(os.path.join(os.path.dirname(f), 'done')):
            rows += list(csv.DictReader(open(f)))
    room = [r for r in rows if r['kind'] == 'room']
    dock = [r for r in rows if r['kind'] == 'dock']
    ok = [r for r in dock if r['result'] == 'success']
    secs = f"{statistics.median(float(r['duration_s']) for r in ok):.0f}" if ok else '-'
    left = f"{statistics.median(float(r['err_xy_m']) for r in dock):.2f}" if dock else '-'
    print(f"{v:16}{len(room):>7}{sum(r['result'] == 'success' for r in room):>9}"
          f"{len(dock):>7}{len(ok):>8}{secs:>8}{left:>13}")
PY
