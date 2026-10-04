#!/usr/bin/env bash
# NAUTILUS simulasyonunu baslatir (varsayilan: GUI, NVIDIA offload).
# Kullanim: scripts/run_sim.sh [gui|nogpu] [rate_hz] [quality: low|medium|high]
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
MODE=${1:-gui}; RATE=${2:-100.0}; QUALITY=${3:-medium}
SCN=$ROOT/scenarios/nautilus.scn
python3 "$ROOT/scripts/gen_scenario.py" > /dev/null   # her seferinde guncel vehicle_derived.yaml'den
source /opt/ros/humble/setup.bash
source ~/stonefish_ws/install/setup.bash
# mesh yollari mutlak; data dizini sadece arguman olarak gerekli
if [ "$MODE" = gui ]; then
  exec env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    ros2 launch stonefish_ros2 stonefish_simulator.launch.py simulation_data:=$ROOT scenario_desc:=$SCN \
      simulation_rate:=$RATE window_res_x:=1600 window_res_y:=900 rendering_quality:=$QUALITY
else
  exec ros2 launch stonefish_ros2 stonefish_simulator_nogpu.launch.py simulation_data:=$ROOT scenario_desc:=$SCN \
    simulation_rate:=$RATE
fi
