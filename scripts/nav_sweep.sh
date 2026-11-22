#!/usr/bin/env bash
# nav_sweep: send the robot to one room with several controller settings and compare them.
#
# Usage (same place as run_batch.sh, from a sourced workspace):
#   scripts/nav_sweep.sh <matrix file> <name>
#
# Each variant is a patched copy of config/nav2_params.yaml, run as its own batch
# <name>_<variant> with no resident and no bag. Settings: ROOM [Bathroom], RESULTS [~/runs].
#   control      the current settings
#   cost10       obstacle cost falls off faster, so a doorway's centre costs less
#   shim         the robot turns on the spot to face its path before driving
#   shim_cost10  both
#   shim_cost10_turn  both, and the shim also does the final turn onto the goal's heading
# ONLY="<variants>" runs just those. Ends with a table: trips, arrivals, time per arrival and
# median distance left to the goal.

set -u

MATRIX=${1:?usage: nav_sweep.sh <matrix file> <name>}
NAME=${2:?usage: nav_sweep.sh <matrix file> <name>}
HERE=$(cd "$(dirname "$0")" && pwd)
export RESULTS=${RESULTS:-$HOME/runs}
export ROOM=${ROOM:-Bathroom} RESIDENT=none TASK=room RETURNS=0 BAG=0
OUT=$RESULTS/nav_variants/$NAME
mkdir -p "$OUT"

VARIANTS=(control cost10)
if ros2 pkg prefix nav2_rotation_shim_controller >/dev/null 2>&1; then
  VARIANTS+=(shim shim_cost10 shim_cost10_turn)
else
  echo "nav2_rotation_shim_controller is not installed: skipping the shim variants"
fi
if [ -n "${ONLY:-}" ]; then
  read -r -a VARIANTS <<< "$ONLY"
fi

python3 - "$HERE/../config/nav2_params.yaml" "$OUT" <<'PY' || exit 1
import copy, os, sys, yaml
base = yaml.safe_load(open(sys.argv[1]))

def cost10(p):
    for name in ('local_costmap', 'global_costmap'):
        p[name][name]['ros__parameters']['inflation_layer']['cost_scaling_factor'] = 10.0

def shim(p):
    c = p['controller_server']['ros__parameters']['FollowPath']
    c['primary_controller'] = c['plugin']
    c['plugin'] = 'nav2_rotation_shim_controller::RotationShimController'
    # Turn rate and acceleration stay inside the PMB2 limits used by MPPI (wz_max, az_max).
    c.update(angular_dist_threshold=0.5, forward_sampling_distance=0.5,
             rotate_to_heading_angular_vel=0.8, max_angular_accel=1.0, simulate_ahead_time=1.0,
             rotate_to_goal_heading=False)

def turn(p):
    p['controller_server']['ros__parameters']['FollowPath']['rotate_to_goal_heading'] = True

for name, changes in {'control': [], 'cost10': [cost10], 'shim': [shim],
                      'shim_cost10': [shim, cost10],
                      'shim_cost10_turn': [shim, cost10, turn]}.items():
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
print(f"\n{'variant':18}{'trips':>6}{'arrived':>9}{'median s':>10}{'slowest s':>11}{'left m':>8}")
for v in variants:
    rows = []
    for f in glob.glob(os.path.join(root, f'{name}_{v}', 'runs', '*', 'events.csv')):
        if os.path.isfile(os.path.join(os.path.dirname(f), 'done')):
            rows += [r for r in csv.DictReader(open(f)) if r['kind'] == 'room']
    ok = [float(r['duration_s']) for r in rows if r['result'] == 'success']
    med, top = (f'{statistics.median(ok):.0f}', f'{max(ok):.0f}') if ok else ('-', '-')
    left = f"{statistics.median(float(r['err_xy_m']) for r in rows):.2f}" if rows else '-'
    print(f'{v:18}{len(rows):>6}{len(ok):>9}{med:>10}{top:>11}{left:>8}')
PY
