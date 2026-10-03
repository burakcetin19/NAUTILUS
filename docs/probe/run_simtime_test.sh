#!/usr/bin/env bash
# use_sim_time testi: kopru /clock yayinlamadigi icin sim ilerliyor mu, damgalar ne oluyor?
# Kullanim: run_simtime_test.sh <true|false>
HERE=$(cd "$(dirname "$0")" && pwd)
ST=${1:-true}
OUT=$HERE/bags/r4_simtime_$ST
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source ~/stonefish_ws/install/setup.bash

ros2 run stonefish_ros2 stonefish_simulator_nogpu "$HOME/stonefish/Tests/Data" "$HERE/probe_phase0.scn" 100.0 \
  --ros-args -p use_sim_time:=$ST > "$OUT/sim.log" 2>&1 &
sleep 8
echo "use_sim_time: $(timeout --foreground 5 ros2 param get /stonefish_simulator_nogpu use_sim_time 2>&1)"
echo "/clock: $(ros2 topic info /clock 2>&1 | tr '\n' ' ')"
timeout --foreground 6 ros2 topic echo /probe/odometry > "$OUT/odom.txt" 2>/dev/null
echo "odometri 6 s: $(grep -c 'frame_id: world_ned' "$OUT/odom.txt") mesaj"
grep -m2 -A2 "stamp:" "$OUT/odom.txt" | grep -E "sec|nanosec" | head -2
grep -m1 -A1 "position:" "$OUT/odom.txt" | tail -1
timeout --foreground 6 ros2 topic echo /probe/debug/physics --field header.stamp.sec > "$OUT/dbg.txt" 2>/dev/null
echo "debug 6 s: $(grep -cE '^[0-9]+$' "$OUT/dbg.txt") mesaj"
timeout --foreground 4 ros2 topic pub -r 20 /probe/thrusters std_msgs/msg/Float64MultiArray "{data: [20.0]}" > /dev/null 2>&1
timeout --foreground 4 ros2 topic echo --once /probe/odometry --field twist.twist.linear.x > "$OUT/u.txt" 2>/dev/null
echo "20 N sonrasi u: $(head -1 "$OUT/u.txt")"
echo "sim log son satir: $(tail -1 "$OUT/sim.log")"
pkill -TERM -f "[l]ib/stonefish_ros2/stonefish_simulator_nogpu"
for i in $(seq 1 10); do pgrep -f "[l]ib/stonefish_ros2/stonefish_simulator" > /dev/null || break; sleep 0.5; done
pgrep -f "[l]ib/stonefish_ros2/stonefish_simulator" > /dev/null && echo "UYARI: sim kapanmadi" || echo "sim kapandi"
