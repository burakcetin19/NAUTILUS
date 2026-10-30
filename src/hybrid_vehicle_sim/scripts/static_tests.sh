#!/usr/bin/env bash
# Faz 2 statik testleri: 9 kosu, nogpu, 100 Hz (roll tekrarlari 500 ve 50 Hz), sifir itki (thrust_cmd.py). ~10 dk.
# Tek kosu: ONLY="surface_roll roll_neutral_50" scripts/static_tests.sh
# Analiz: python3 scripts/analyze_static.py
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
Z_EQ=$(python3 -c "import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))['surface']['axis_depth'])" \
       "$ROOT/config/vehicle_derived.yaml")
Z_HEAVE=$(python3 -c "print($Z_EQ + 0.01)")

run() {
  if [ -n "$ONLY" ] && [[ " $ONLY " != *" $1 "* ]]; then return; fi
  echo "=== $1 ==="; "$HERE/run_case.sh" "$@"
}
run depth_release 25 --depth 5.0                                   # S1, S3
run depth_default 20 --depth 5.0 --default-hydro                   # S2
run surface       90 --depth "$Z_HEAVE"                            # S4: denge + yuzey heave periyodu
run surface_pitch 40 --depth "$Z_EQ" --rpy 0 2 0                   # S4: yuzey pitch periyodu
run roll_neutral  30 --depth 5.0 --rpy 10 0 0 --neutral            # S5
run pitch_neutral 45 --depth 5.0 --rpy 0 5 0 --neutral             # S5
RATE=500.0 run roll_neutral_500 30 --depth 5.0 --rpy 10 0 0 --neutral   # S5: roll buyumesi sim adimina bagli mi
RATE=50.0  run roll_neutral_50  30 --depth 5.0 --rpy 10 0 0 --neutral   # S5: 50 Hz'de tutma gecikmesi yok (P = 1)
run surface_roll  40 --depth "$Z_EQ" --rpy 5 0 0                   # S4: yuzeyde roll
