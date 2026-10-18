#!/usr/bin/env bash
# Stop every simulation, Nav2 and RViz process, then reset ROS's node list.
pkill -f "ros2 launch"        2>/dev/null
pkill -f "ros2 run"           2>/dev/null
pkill -f rviz2                2>/dev/null
pkill -f "gz sim"             2>/dev/null
pkill -f parameter_bridge     2>/dev/null
pkill -f people_publisher.py  2>/dev/null
pkill -f social_zones.py      2>/dev/null
pkill -f send_goal.py         2>/dev/null
sleep 3
pkill -9 -f "gz sim|rviz2|nav2|_server|lifecycle_manager|robot_state_publisher|static_transform_publisher" 2>/dev/null
ros2 daemon stop  >/dev/null 2>&1   # forget old, already-closed nodes
ros2 daemon start >/dev/null 2>&1
echo "Stopped. Nodes still running:"
ros2 node list
