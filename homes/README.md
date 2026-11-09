# Homes

Gazebo models and worlds of the scanned homes, one folder each, made by
`tools/scan_to_world.py` from the private scans in `Scans/`. The folders are git-ignored
because they are real people's homes.

```bash
python3 tools/scan_to_world.py "Scans/Guest_Flat/Flat/OBJ/Main Guest Flat.zip" guest_flat \
  --floorplan "Scans/Guest_Flat/Flat/Floorplan/Main Guest Flat.csv"
colcon build --packages-select elderly_care_robot_sim   # from ~/ros2_ws
ros2 launch elderly_care_robot_sim robot_in_house.launch.py \
  world:=../homes/guest_flat/guest_flat.world x:=-2.35 y:=0.59 yaw:=0.00
# second terminal: Nav2 on the home's generated map
ros2 launch elderly_care_robot_sim navigation.launch.py world:=../homes/guest_flat/guest_flat.world \
  map:=$(ros2 pkg prefix elderly_care_robot_sim)/share/elderly_care_robot_sim/homes/guest_flat/map.yaml \
  x:=-2.35 y:=0.59 yaw:=0.00
# third terminal: jobs and dock calls
ros2 run elderly_care_robot_sim task_runner.py --ros-args -p use_sim_time:=true \
  -p home:=guest_flat -p task:=patrol -p returns:=5 -p seed:=1 -p social:=false
```

The converter prints the launch pose for each home: the dock, so every run starts there
(the world has a green dock plate at it). The dock is a suggestion: if you move it in
`annotations.yaml`, also move the `<model name='dock'>` pose in the world file. It also
writes `preview.png`, a top view to check by eye: obstacles black, doorways blue, floor
TIAGo's centre can reach green, other floor inside white, outside grey, most open spot red.

There are two maps: `map.pgm` is what the robot's laser sees (AMCL localises on it) and
`costmap.pgm` adds each furniture box (Nav2 plans on it). Pass `map.yaml`; the navigation
launch picks up `costmap.yaml` next to it.

Each home also gets `annotations.yaml` (rooms named from the floor-plan CSV, doorways and
the rooms they join, passages narrower than 1.2 m, a suggested dock pose, resident spots,
patrol goals covering every room the robot's centre can reach, and `unreachable_rooms`),
`rooms.pgm` (room id per map cell) and `annotations.png` to check them: rooms coloured,
doorways blue, narrow passages orange, dock red, resident spots purple. The converter
prints which colour is which room. Only the CSV's per-room area and dimension rows are
read, never its name row, which can hold the address.

The converter also gives each home a resident (`tools/resident_routine.py`): a looped,
compressed day walking bedroom → bathroom → day rooms and back at 0.65 m/s, pausing 30 s
at each stop, written into the home's world as `<actor name='resident'>` and drawn in
`routine.png`. To change the pace or the pauses without reconverting:

```bash
python3 tools/resident_routine.py homes/guest_flat --speed 0.6 --dwell 45
```
