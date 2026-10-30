#!/usr/bin/env bash
# Test kosusu (nogpu): senaryo varyanti uret -> bag kaydi -> sim -> sure dolunca kapat.
# Kayit sim'den ONCE baslar: serbest birakma ve salinim testlerinde ilk adimlar kaybolmasin.
# Kullanim: scripts/run_case.sh <ad> <sure_s> [gen_scenario argumanlari...]
#   ornek:  scripts/run_case.sh depth_release 25 --depth 5
# Ortam:   RATE (varsayilan 100.0), BAGDIR (bags/ altinda, varsayilan faz2),
#          THRUST (sabit itki vektoru [N], varsayilan "0 0 0 0"; sim'den once yayina baslar)
set -eo pipefail
NAME=$1; DUR=$2; shift 2
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RATE=${RATE:-100.0}
OUT=$ROOT/bags/${BAGDIR:-faz2}
SCN=$ROOT/scenarios/tests/$NAME.scn
LOG=$OUT/$NAME.sim.log
SIM="[l]ib/stonefish_ros2/stonefish_simulator"
pgrep -f "$SIM" > /dev/null && { echo "baska bir stonefish_simulator calisiyor; once kapatin"; exit 1; }
mkdir -p "$OUT" "$(dirname "$SCN")"; rm -rf "${OUT:?}/$NAME"
python3 "$ROOT/scripts/gen_scenario.py" --out "$SCN" "$@" > /dev/null
source /opt/ros/humble/setup.bash
source ~/stonefish_ws/install/setup.bash
NS=$(python3 -c "import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))['name'])" "$ROOT/config/vehicle_derived.yaml")
TOPICS="/$NS/debug/physics /$NS/odometry /$NS/imu /$NS/pressure /$NS/thruster_state /$NS/thrusters"

ros2 bag record -o "$OUT/$NAME" $TOPICS > "$OUT/$NAME.bag.log" 2>&1 &
BAGPID=$!
# sifir itki sim'den ONCE yayinda: SimpleThruster sThrust/sTorque'u baslatmiyor (thrust_cmd.py)
python3 "$ROOT/scripts/thrust_cmd.py" --topic "/$NS/thrusters" --vector ${THRUST:-0 0 0 0} > "$OUT/$NAME.cmd.log" 2>&1 &
CMDPID=$!
sleep 2
ros2 launch stonefish_ros2 stonefish_simulator_nogpu.launch.py simulation_data:=$ROOT scenario_desc:=$SCN \
  simulation_rate:=$RATE > "$LOG" 2>&1 &
for i in $(seq 1 60); do grep -q "IC problem solved" "$LOG" && break; sleep 0.5; done
if ! grep -q "IC problem solved" "$LOG"; then
  echo "sim baslamadi"; tail -20 "$LOG"; pkill -TERM -f "$SIM" || true; kill -TERM $CMDPID; kill -INT $BAGPID; exit 1
fi
sleep "$DUR"
# betik icindeki arka plan surecleri SIGINT'i yok sayar -> SIGTERM (run_probe.sh ile ayni)
pkill -TERM -f "$SIM" || true
for i in $(seq 1 20); do pgrep -f "$SIM" > /dev/null || break; sleep 0.5; done
pgrep -f "$SIM" > /dev/null && echo "UYARI: sim kapanmadi"
sleep 1
kill -TERM $CMDPID; wait $CMDPID || true
kill -INT $BAGPID; wait $BAGPID || true
# "exit code -15" satiri bizim SIGTERM'imiz; disindaki hata satirlari uyari
if grep -iE "error|failed|could not" "$LOG" | grep -qv "exit code -15"; then echo "UYARI: sim log'unda hata satiri var: $LOG"; fi
ros2 bag info "$OUT/$NAME" | grep -E "Duration|Messages|Topic:"
