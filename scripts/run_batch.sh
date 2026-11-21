#!/usr/bin/env bash
# run_batch: run a list of task_runner scenarios one after another, headless and unattended.
#
# Usage (from a sourced workspace, inside the DCV session so Gazebo can render the laser):
#   PURPOSE="what this batch tests" scripts/run_batch.sh <matrix file> [batch name]
# PURPOSE is required the first time; resuming a batch does not need it.
#
# Matrix file: one run per line, "home social seed" (# starts a comment), e.g.
#   guest_flat false 1
#   guest_flat true  1
#
# Settings (environment variables, defaults in brackets):
#   TASK [patrol]  RETURNS [3]  WALL_LIMIT seconds per run [3600]  BAG 1 = record a rosbag [1]
#   RESULTS root folder [~/runs]
#   SIM_WAIT [240] / NAV_WAIT [180]  seconds to wait for the robot / for Nav2 to come up
#   SETTLE [10]  seconds to wait after each stage is up
#
# Order per run: sim -> robot's wheel odometry publishing -> Nav2 -> amcl and bt_navigator
# active -> bag and task_runner. A stage that does not come up fails the run (no_sim, no_nav2
# in the summary) and the batch moves on.
#
# Output, in RESULTS/batches/<batch>/:
#   batch.yaml    purpose, date, git commit, resident, task settings, matrix
#   summary.tsv   one line per run
#   runs/<home>_<base|social>_s<seed>/
#     events.csv, trace.csv, run.yaml  from task_runner
#     logs/                            sim, nav2, task and bag output
#     bag/                             /ground_truth/odom /people/odom /task_events
#     done                             written only when task_runner finished normally
# Runs with a done file are skipped, so a stopped batch resumes when started again with the
# same name. It refuses to resume if the code or settings changed: use a new batch name, so
# runs that cannot be compared never share a batch.
#
# Warning: it stops every Gazebo, Nav2 and RViz process on the machine before each run.

set -u

MATRIX=${1:?usage: run_batch.sh <matrix file> [batch name]}
BATCH=${2:-$(basename "${MATRIX%.*}")}
TASK=${TASK:-patrol}
RETURNS=${RETURNS:-3}
WALL_LIMIT=${WALL_LIMIT:-3600}
BAG=${BAG:-1}
RESULTS=${RESULTS:-$HOME/runs}
SIM_WAIT=${SIM_WAIT:-240}
NAV_WAIT=${NAV_WAIT:-180}
SETTLE=${SETTLE:-10}
BATCH_DIR=$RESULTS/batches/$BATCH
LOG_DIR=$BATCH_DIR/runs
SUMMARY=$BATCH_DIR/summary.tsv
REPO=$(cd "$(dirname "$0")/.." && pwd)
COMMIT=$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)
DIRTY=$([ -n "$(git -C "$REPO" status --porcelain 2>/dev/null)" ] && echo true || echo false)

[ -f "$MATRIX" ] || { echo "No matrix file: $MATRIX"; exit 1; }
SHARE=$(ros2 pkg prefix elderly_care_robot_sim 2>/dev/null)/share/elderly_care_robot_sim
[ -d "$SHARE" ] || { echo "elderly_care_robot_sim not found: source install/setup.bash"; exit 1; }

write_record() {
  python3 - "$BATCH_DIR/batch.yaml" "$MATRIX" "$BATCH" "${PURPOSE:-}" "$TASK" "$RETURNS" "$BAG" \
    "$COMMIT" "$DIRTY" <<'EOF'
import datetime, os, sys, yaml
path, matrix, batch, purpose, task, returns, bag, commit, dirty = sys.argv[1:]
lines = [' '.join(l.split()) for l in open(matrix) if l.strip() and not l.lstrip().startswith('#')]
new = {'batch': batch, 'purpose': purpose,
       'created': datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
       'git_commit': commit, 'git_dirty': dirty == 'true', 'resident': 'scripted', 'task': task,
       'returns': int(returns), 'bag': bag == '1', 'matrix': lines}
if os.path.isfile(path):
    old = yaml.safe_load(open(path))
    changed = [k for k in new if k not in ('purpose', 'created') and old.get(k) != new[k]]
    if changed:
        sys.exit(f"Batch '{batch}' was started with different {', '.join(changed)}. "
                 'Use a new batch name.')
else:
    if not purpose:
        sys.exit(f"New batch '{batch}': set PURPOSE=\"what this batch tests\".")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    yaml.safe_dump(new, open(path, 'w'), sort_keys=False, allow_unicode=True)
EOF
}

write_record || exit 1
mkdir -p "$LOG_DIR"
[ -f "$SUMMARY" ] || printf 'run_id\texit\twall_s\tdocked\tdock_calls\tpatrol_ok\tpatrol_goals\n' > "$SUMMARY"

PGIDS=()

cleanup() {
  for g in "${PGIDS[@]}"; do kill -INT -- "-$g" 2>/dev/null; done
  sleep 8
  for g in "${PGIDS[@]}"; do kill -KILL -- "-$g" 2>/dev/null; done
  PGIDS=()
  # Gazebo and Nav2 children sometimes outlive their launch; a leftover server or /clock
  # would feed the next run.
  pkill -9 -f "gz sim|parameter_bridge|rviz2|nav2|_server|lifecycle_manager|robot_state_publisher|static_transform_publisher|people_publisher.py|social_zones.py|task_runner.py|ros2 bag record" 2>/dev/null
  sleep 2
  ros2 daemon stop >/dev/null 2>&1
  ros2 daemon start >/dev/null 2>&1
}
trap 'cleanup; exit 130' INT TERM

start() {   # start <log file> <command...>: own process group, so cleanup can stop all of it
  setsid "${@:2}" > "$1" 2>&1 &
  PGIDS+=($!)
  disown $!
}

active() { ros2 lifecycle get "/$1" 2>/dev/null | grep -q '^active'; }

wait_until() {   # wait_until <seconds> <command...>: retry every 2 s until it succeeds
  local end=$(( SECONDS + $1 )); shift
  until "$@"; do
    [ $SECONDS -ge $end ] && return 1
    sleep 2
  done
}

fail() {   # fail <run> <code> <message>: log a run that never started its task
  echo "  $3"
  cleanup
  printf '%s\t%s\t%s\t\t\t\t\n' "$1" "$2" $(( $(date +%s) - T0 )) >> "$SUMMARY"
}

dock_pose() {
  python3 -c "import sys, yaml; d = yaml.safe_load(open(sys.argv[1]))['dock']; print(d['x'], d['y'], d['yaw'])" \
    "$SHARE/homes/$1/annotations.yaml"
}

summarise() {   # summarise <events.csv>: docked, dock calls, patrol successes, patrol goals
  python3 - "$1" <<'EOF'
import csv, sys
try:
    rows = list(csv.DictReader(open(sys.argv[1])))
except OSError:
    rows = []
dock = [r for r in rows if r['kind'] == 'dock']
patrol = [r for r in rows if r['kind'] == 'patrol' and r['result'] != 'interrupted']
ok = lambda rs: sum(r['result'] == 'success' for r in rs)
print(ok(dock), len(dock), ok(patrol), len(patrol), sep='\t')
EOF
}

cleanup
# The loop must run in this shell (not a pipe) so the INT trap sees PGIDS, and read from fd 3
# so ros2 commands inside cannot swallow the matrix from stdin.
while read -r HOME_ID SOCIAL SEED _ <&3; do
  [ -n "${HOME_ID:-}" ] || continue
  MODE=$([ "$SOCIAL" = true ] && echo social || echo base)
  RUN=${HOME_ID}_${MODE}_s${SEED}
  OUT=$LOG_DIR/$RUN
  if [ -f "$OUT/done" ]; then echo "skip $RUN (done)"; continue; fi
  if ! POSE=$(dock_pose "$HOME_ID"); then echo "skip $RUN: no dock in annotations.yaml"; continue; fi
  read -r X Y YAW <<< "$POSE"
  rm -rf "$OUT"; mkdir -p "$OUT/logs"
  WORLD=../homes/$HOME_ID/$HOME_ID.world
  echo "$(date +%H:%M:%S) start $RUN (dock $X $Y $YAW)"
  T0=$(date +%s)

  start "$OUT/logs/sim.log" ros2 launch elderly_care_robot_sim robot_in_house.launch.py \
    world:="$WORLD" x:="$X" y:="$Y" yaw:="$YAW" headless:=True
  # Wheel odometry only publishes once the base controller is active, the last thing the sim
  # brings up. AMCL started earlier would miss sim time or the odom frame.
  echo "  waiting for the robot (up to ${SIM_WAIT}s)"
  # Without the type, echo exits at once if the topic does not exist yet instead of waiting.
  if ! timeout "$SIM_WAIT" ros2 topic echo --once /mobile_base_controller/odom nav_msgs/msg/Odometry \
      > /dev/null 2>&1; then
    fail "$RUN" no_sim "no /mobile_base_controller/odom after ${SIM_WAIT} s, see $OUT/logs/sim.log"
    continue
  fi
  sleep "$SETTLE"
  start "$OUT/logs/nav2.log" ros2 launch elderly_care_robot_sim navigation.launch.py social:="$SOCIAL" \
    world:="$WORLD" map:="$SHARE/homes/$HOME_ID/map.yaml" x:="$X" y:="$Y" yaw:="$YAW" rviz:=false
  echo "  waiting for Nav2 (up to ${NAV_WAIT}s)"
  if ! { wait_until "$NAV_WAIT" active amcl && wait_until "$NAV_WAIT" active bt_navigator; }; then
    fail "$RUN" no_nav2 "Nav2 not active after ${NAV_WAIT} s, see $OUT/logs/nav2.log"
    continue
  fi
  sleep "$SETTLE"
  # No /clock: message headers carry sim time, and /clock was 750k of 780k messages per run.
  [ "$BAG" = 1 ] && start "$OUT/logs/bag.log" ros2 bag record -o "$OUT/bag" \
    /ground_truth/odom /people/odom /task_events
  echo "  running task_runner (limit ${WALL_LIMIT}s)"

  timeout "$WALL_LIMIT" ros2 run elderly_care_robot_sim task_runner.py --ros-args \
    -p use_sim_time:=true -p home:="$HOME_ID" -p task:="$TASK" -p returns:="$RETURNS" \
    -p seed:="$SEED" -p social:="$SOCIAL" -p run_id:="$RUN" -p log_dir:="$LOG_DIR" \
    > "$OUT/logs/task.log" 2>&1
  CODE=$?
  [ $CODE = 0 ] && touch "$OUT/done"
  cleanup
  WALL=$(( $(date +%s) - T0 ))
  printf '%s\t%s\t%s\t%s\n' "$RUN" "$CODE" "$WALL" "$(summarise "$OUT/events.csv")" >> "$SUMMARY"
  echo "$(date +%H:%M:%S) end   $RUN exit $CODE ($([ $CODE = 124 ] && echo 'wall limit' || echo "${WALL} s"))"
done 3< <(grep -v '^[[:space:]]*#' "$MATRIX")
echo "Summary: $SUMMARY"
