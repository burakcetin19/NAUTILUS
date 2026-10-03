#!/usr/bin/env bash
# Probe kosusu: sim baslat -> bag kaydi -> itki basamaklari -> kapat.
# Kullanim: run_probe.sh <gui|nogpu> <rate_hz> <senaryo.scn> <bag_adi> "<thrust_steps argumanlari>" <bag topic'leri...>
set -eo pipefail
MODE=$1; RATE=$2; SCN=$3; BAG=$4; STEPS=$5; shift 5; TOPICS="$*"
HERE=$(cd "$(dirname "$0")" && pwd)
LOG=$HERE/bags/$BAG.sim.log
mkdir -p "$HERE/bags"; rm -rf "$HERE/bags/$BAG"
source /opt/ros/humble/setup.bash
source ~/stonefish_ws/install/setup.bash
DATA=$HOME/stonefish/Tests/Data

if [ "$MODE" = gui ]; then
  __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch stonefish_ros2 stonefish_simulator.launch.py simulation_data:=$DATA scenario_desc:=$SCN \
    simulation_rate:=$RATE window_res_x:=1200 window_res_y:=800 rendering_quality:=low > "$LOG" 2>&1 &
else
  ros2 launch stonefish_ros2 stonefish_simulator_nogpu.launch.py simulation_data:=$DATA scenario_desc:=$SCN \
    simulation_rate:=$RATE > "$LOG" 2>&1 &
fi
for i in $(seq 1 40); do grep -q "IC problem solved" "$LOG" && break; sleep 0.5; done
grep -q "IC problem solved" "$LOG" || { echo "sim baslamadi"; tail -20 "$LOG"; exit 1; }
sleep 2

ros2 bag record -o "$HERE/bags/$BAG" $TOPICS > "$HERE/bags/$BAG.bag.log" 2>&1 &
BAGPID=$!
sleep 3
python3 "$HERE/thrust_steps.py" $STEPS
sleep 1
kill -INT $BAGPID; wait $BAGPID || true
# betik icindeki arka plan surecleri SIGINT'i yok sayar -> SIGTERM
pkill -TERM -f "[l]ib/stonefish_ros2/stonefish_simulator" || true
for i in $(seq 1 20); do pgrep -f "[l]ib/stonefish_ros2/stonefish_simulator" > /dev/null || break; sleep 0.5; done
pgrep -f "[l]ib/stonefish_ros2/stonefish_simulator" > /dev/null && echo "UYARI: sim kapanmadi" || echo "sim kapandi"
ros2 bag info "$HERE/bags/$BAG" | grep -E "Duration|Messages|Topic:"
