# Homes

Gazebo models and worlds of the scanned homes, one folder each, made by
`tools/scan_to_world.py` from the private scans in `Scans/`. The folders are git-ignored
because they are real people's homes.

```bash
python3 tools/scan_to_world.py "Scans/Guest_Flat/Flat/OBJ/Main Guest Flat.zip" guest_flat \
  --floorplan "Scans/Guest_Flat/Flat/Floorplan/Main Guest Flat.csv"
colcon build --packages-select elderly_care_robot_sim   # from ~/ros2_ws
ros2 launch elderly_care_robot_sim robot_in_house.launch.py \
  world:=../homes/guest_flat/guest_flat.world x:=-0.55 y:=0.64 yaw:=0
# second terminal: Nav2 on the home's generated map
ros2 launch elderly_care_robot_sim navigation.launch.py world:=../homes/guest_flat/guest_flat.world \
  map:=$(ros2 pkg prefix elderly_care_robot_sim)/share/elderly_care_robot_sim/homes/guest_flat/map.yaml \
  x:=-0.55 y:=0.64 yaw:=0
```

The converter prints the spawn `x` and `y` for each home and writes `preview.png`, a top
view to check by eye: obstacles black, doorways blue, floor TIAGo's centre can reach
green, other floor inside white, outside grey, spawn red.

Each home also gets `annotations.yaml` (rooms named from the floor-plan CSV, doorways and
the rooms they join, passages narrower than 1.2 m, a suggested dock pose, resident spots),
`rooms.pgm` (room id per map cell) and `annotations.png` to check them: rooms coloured,
doorways blue, narrow passages orange, dock red, resident spots purple. The converter
prints which colour is which room. Only the CSV's per-room area and dimension rows are
read, never its name row, which can hold the address.
