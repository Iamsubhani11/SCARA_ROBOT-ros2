#!/bin/bash
# Every automatic test this computer can run, then one summary.
# Run it before every commit - it is what keeps the bugs in docs/FIXED_BUGS.md from coming back.
#
#   bash tools/run_all_tests.sh            # everything (MuJoCo / Gazebo parts take a few minutes)
#   bash tools/run_all_tests.sh --quick    # only the fast ones (about a minute)
#
# Tests whose tools are missing are SKIPPED (and say what to install), never counted as passed.
cd "$(dirname "$0")/.." || exit 1
QUICK=0; [ "$1" = "--quick" ] && QUICK=1
LOG=$(mktemp -d /tmp/scara_tests.XXXX)
declare -a NAMES RES
have_py() { python3 -c "import $1" 2>/dev/null; }

run() {   # run <name> <command...>
  local name=$1; shift
  printf '%-34s ' "$name"
  local out="$LOG/$(echo "$name" | tr ' /' '__').log"
  if "$@" > "$out" 2>&1 && ! grep -qE '^FAIL |[0-9]+ FAILED|"PASS": false|Traceback \(most recent' "$out"; then
    echo "pass"; NAMES+=("$name"); RES+=("pass")
  else
    echo "FAIL   (log: $out)"; NAMES+=("$name"); RES+=("FAIL")
    grep -E '^FAIL |FAILED|Error|error:' "$out" | head -5 | sed 's/^/      /'
  fi
}
skip() { printf '%-34s skipped - %s\n' "$1" "$2"; NAMES+=("$1"); RES+=("skip"); }

echo "SCARA tests - logs in $LOG"
run "static checks (old bugs)"   python3 tools/test_static.py
run "robot description"          python3 tools/check_description.py
run "kinematics"                 python3 common/scara_kinematics.py
run "safety filter"              python3 common/scara_safety.py
run "robot protocol"             python3 common/scara_twin_protocol.py
if have_py numpy; then run "gestures (MediaPipe logic)" python3 tools/test_gesture_core.py
else skip "gestures (MediaPipe logic)" "pip install numpy"; fi
if command -v g++ >/dev/null; then run "Arduino firmware on the PC" python3 tools/test_firmware_host.py
else skip "Arduino firmware on the PC" "install g++"; fi

if have_py playwright; then
  HAND=$LOG/hand.y4m; python3 tools/make_hand_video.py "$HAND" > /dev/null
  run "app + website (offline)"   python3 tools/test_app_v3.py --hand-video "$HAND" --shots "$LOG"
  run "app basics"                python3 tools/test_app.py
  if have_py cv2; then run "webcam cell as a twin" python3 tools/test_hw_twin.py --shots "$LOG"
  else skip "webcam cell as a twin" "pip install opencv-python"; fi
else
  skip "app + website (offline)" "pip install playwright && playwright install chromium"
  skip "app basics" "playwright"; skip "webcam cell as a twin" "playwright"
fi

if [ $QUICK = 1 ]; then echo "(--quick: MuJoCo and Gazebo runs left out)"
else
  if have_py mujoco; then
    have_py playwright && run "digital twin (MuJoCo + robot)" env SHOTS="$LOG" python3 tools/test_twin_e2e.py \
      || skip "digital twin (MuJoCo + robot)" "playwright"
    run "MuJoCo manual picking"     python3 tools/sim_test_manual.py
    run "MuJoCo sorting cell"     env MODE=sort python3 tools/sim_test_sorting.py
    run "MuJoCo command cell"     env MODE=command python3 tools/sim_test_sorting.py
    have_py cv2 && run "webcam cell vs MuJoCo" python3 tools/test_hw_cell.py command \
      || skip "webcam cell vs MuJoCo" "pip install opencv-python"
  else
    for t in "digital twin (MuJoCo + robot)" "MuJoCo manual picking" "MuJoCo sorting cell" "MuJoCo command cell" "webcam cell vs MuJoCo"; do
      skip "$t" "pip install mujoco"; done
  fi
  if command -v ros2 >/dev/null && ros2 pkg prefix scara_description >/dev/null 2>&1; then
    run "Gazebo + twin"             python3 tools/test_gazebo.py
  else skip "Gazebo + twin" "source ROS 2 and this workspace (colcon build)"; fi
fi

p=0; f=0; s=0
for r in "${RES[@]}"; do case $r in pass) p=$((p+1));; FAIL) f=$((f+1));; skip) s=$((s+1));; esac; done
echo "-----------------------------------------------"
echo "passed $p   failed $f   skipped $s"
[ $f = 0 ] && echo "ALL TESTS THAT COULD RUN PASSED" || echo "SOMETHING FAILED - see the logs above"
exit $f
