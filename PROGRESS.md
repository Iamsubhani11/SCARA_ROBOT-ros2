# Progress of the 3.3 update – stopped on 28 October 2026

Work is on the git branch `update-3.3` (not pushed). This file follows the sections of the task
prompt. "Done" means done **and** checked by a test that passed; anything not checked says so.

## Where it stopped

In the middle of **section 4 – belt tracking in Gazebo**. Belt tracking works and is tested
offline (app) and in MuJoCo, but **in Gazebo every tracked pick still misses** (see "Open errors" 1).
The last change (the belt speed measured from all pucks seen by the camera, `BeltSpeed` in
`common/scara_sorter_core.py`) passes MuJoCo but was **not yet tried in Gazebo** – that run was
stopped. Nothing was left running.

## 0. Setup – done

* The folder was not a git repository. The GitHub clone's `.git` (release 3.1.0) was put into the
  folder and the branch `update-3.3` was made. Commit 1 on the branch is the 3.2.1 release exactly as
  delivered (from `~/Downloads/SCARA_ROBOT-ros2.zip`), commit 2 is this work – so `git diff` between
  them shows only what changed. Commit 1 also records that `scripts/` and `firmware/` (the 3.0 layout)
  are gone, as the 3.1 notes say to do; they are still in the git history.
* `SCARA_SIMULATION.zip` was a zip, not a folder in the repo: it is unpacked next to the repo in
  `~/Downloads/SCARA_ROBOT-ros2/SCARA_SIMULATION` (it differs from the named version only in the name
  files, pictures, README and CHANGELOG). **Nothing was synced to it yet.**
* Thesis / report: only the PDFs were found (`SCARA_Thesis.pdf` 97 pages, `SCARA_Project_Report.pdf`
  43 pages). **`SCARA_Thesis_and_Report_LaTeX.zip` and `AI_text_check.zip` are not on this PC.**

## 1. Rules – kept

New branch, nothing pushed, no files deleted, no videos or build folders in the repo
(`build/ install/ log/` are in `.gitignore`), the baseline test run was saved before the first change,
every fixed bug has a line in `docs/FIXED_BUGS.md` and a test that fails when it comes back, the
joysticks were only changed where a test showed a bug (#34, see below).

## 2. Environment – mostly done

Installed by me: `playwright` (pip `--user`) + Chromium for it; `~/tools/apktool.jar` (2.9.3) and
`~/tools/uber-apk-signer.jar` (1.3.0) from their GitHub releases; MediaPipe tasks-vision 1.1.1 (npm
package, copied into `app/ui/lib/mediapipe`). Already there: ROS 2 Humble, Gazebo Harmonic 8.15,
ros_gz + gz_ros2_control (Harmonic builds in `~/gz_ros2_control_ws`), MuJoCo 3.12, OpenCV 4.9, numpy,
mediapipe 0.10.14 (Python), node 22, Java 11. **Not installed:** texlive (pdflatex), torch.

`colcon build --symlink-install` works.

**Baseline before any change** (`bash tools/run_all_tests.sh`, full): **13 passed, 4 failed** –
app + website (sliders, test timing), app basics (bugs #34, #35), digital twin (bug #33), Gazebo
(bug #36). All four are fixed now.

Run by hand: done through the tests (Gazebo command / sort / manual worlds headless through the
server, MuJoCo sort / command / table, the website offline and against MuJoCo). **Not done by hand
with windows:** `gazebo.launch.py` with the GUI, `launch_mujoco.py --show`, keyboard / gamepad teleop
programs, PC gesture program with a webcam, `scara_ask.py --voice` (needs a microphone / a person).

## 3. Gesture control in the app / website – done (except a real phone)

* MediaPipe Hand Landmarker (21 points, 1 or 2 hands) bundled offline in `app/ui/lib/mediapipe`
  (21 MB: JS, SIMD wasm, float16 model); new `app/ui/handmp.js`. Palm = left joystick with the same
  still zone, fist / open hand = gripper, pinch = right joystick, two hands = left + right joystick,
  a hand only drives after it has been in its circle. Skeleton drawn on the picture.
* The old colour tracker is kept as an automatic fallback (and a ✋ switch); the window says which
  one runs, and **why** the camera did not start (refused, plain http from another computer, busy,
  no camera, too dark / bright).
* Android: the permission flow already existed; a refused permission is now shown in the window.
  APK rebuilt (11.1 MB, version 3.3.0 / code 8), signed with the **same certificate as 3.2.1**
  (SHA-256 1e08a903…), so it installs over it.
* Tests: `tools/test_gesture_mediapipe.py` (video made from real hand photos in `tools/testdata/`,
  `make_hand_video.py --real`), `test_app_v3.py` checks the fallback.
* **Not checked:** the APK on a real Android phone (no phone / emulator here) – please try the
  Gesture button on your phone.

## 4. New features – mostly done

| feature | offline app | MuJoCo | Gazebo | test |
|---|---|---|---|---|
| Draw a path (FK / IK tab, shapes, drawn) | ✔ | ✔ (same `move` command) | ✔ (same command) | `test_new_features.py` (star followed within 0.9 mm) |
| Programs: save / load / export / import | ✔ | ✔ (`teach_set`, `save_program`) | ✔ (server) | `test_new_features.py`, `test_twin_e2e.py` |
| Belt tracking (command cell, Jacobian) | ✔ | ✔ | **✘ misses** | `test_new_features.py`, `MuJoCo belt tracking`, `test_gazebo.py` (fails) |
| Demo for the review | ✔ | not run end to end | not run end to end | `test_new_features.py` (steps 1–2) |
| Hardware bring-up check `real_robot/scara_bringup.py` + `docs/HARDWARE_GUIDE.md` | – | – | – | `--fake --yes` in `run_all_tests.sh` |

Belt tracking in the **sorting** cell is switched off on purpose: in tests it lost pucks there (every
puck is wanted, one every 2.5 s). Version bumped to 3.3.0 everywhere (a static test checks it).

## 5. README and docs – done for the named version

README (modes table "app / website or terminal", MediaPipe, paths, programs, belt tracking, demo,
hardware guide link, testing, troubleshooting incl. "Gazebo crashes"), `app/README.md`, folder
READMEs, CHANGELOG 3.3.0, FIXED_BUGS #33–#38, `docs/HARDWARE_GUIDE.md`, new pictures in `docs/media`
(`gesture_mediapipe.png`, `app_path.png`, `app_tracking.png`, made by `tools/make_v33_media.py`).
**CHANGELOG still says "the summary is in CHANGELOG.md" for the final test run – add it after the run.**

## 6. Thesis and project report – not started

* Needs the LaTeX sources (not found) – where is `SCARA_Thesis_and_Report_LaTeX.zip`?
* I will update both with the new work and make the English plain and natural, check the TikZ
  arrows and rebuild with pdflatex. I will **not** tune the text against an AI detector or use a
  "humanizer" – both the thesis and a journal paper need an honest declaration of AI help (your
  university's and the journal's rules).
* Journal paper (your second prompt): not started. Plan: `research_paper_q1/` with a literature review
  of real, checked papers, results only from our own test logs (simulation results marked as
  simulation, no hardware results until the robot is built), journal shortlist checked on the
  publishers' pages.

## 7. Finish – not done

Still to do: full `run_all_tests.sh` on the named version, sync everything to `SCARA_SIMULATION`
(without the name; rebuild its APK and pictures there), full tests there, `grep -r SUBHANI` in the
plain folder, the two release zips + PDFs + LaTeX in `release/` outside the repo, final `git status`.

## Last test results (named version, before stopping)

* `run_all_tests.sh --quick`: all passed (static, description, kinematics, safety, protocol, gestures,
  firmware, app + website, app basics, gesture camera + gamepads, MediaPipe, webcam cell as a twin).
* `test_new_features.py`: all 17 checks passed. `test_gesture_mediapipe.py`: all passed.
* `test_twin_e2e.py` (MuJoCo + software robot): all passed.
* `sim_test_sorting.py` sort (normal): passed; command + `TRACK=1`, seeds 0 and 1: passed.
* `test_gazebo.py`: everything passed **except** "Gazebo: red puck(s) picked from the moving belt".
* Not re-run after the last edits: the full suite (MuJoCo picking / sorting, webcam cell vs MuJoCo).

## Bugs found and fixed (details in docs/FIXED_BUGS.md)

* #33 the twin test hid `pip --user` packages (new `HOME`)
* #34 XY drive: gripper turned by itself and the arm kept moving after the stick was let go
* #35 tilt drive did nothing in new Chrome / Android WebView
* #36 **Gazebo picked nothing / no pucks after `colcon build --symlink-install`** (shared code not
  found through symlinks) – this was also in your `~/Scara_robot` logs: it is the "Gazebo crashes"
* #37 Gazebo controllers sometimes did not start (spawner timeout, old Gazebo still running)
* #38 app gesture control did nothing with the colour tracker and gave no reason
* also: the kinematics self-test printed but never checked; the Android bridge reported 3.1.0;
  the offline sim's finger guard stopped a gripper closing on a slightly off-centre puck.

## Open errors / to check

1. **Gazebo belt tracking misses every puck.** Measured with a probe: the pad comes down 12–20 mm
   behind the puck along the belt and the gap grows, so a finger lands on the puck's rim. Two causes
   found: Gazebo's belt moves the pucks about 10 % faster than the 0.08 m/s it is told, and its camera
   loses the puck behind the waiting arm. Done so far: camera time stamps passed with every detection
   (`gazebo/scara_vision_node.py`), belt speed measured from all pucks (`BeltSpeed`). Next: run the
   probe again (`/tmp/.../gz_probe.sh` – recreate it from the conversation), if still off try waiting
   further from the belt or a slower meeting.
2. `tools/test_gazebo.py` will fail on the belt-tracking check until 1 is fixed.
3. Not tried on real hardware or a real phone: the APK, MediaPipe on Android, the bring-up check.
4. `common/scara_sorter_core.py` and `tools/test_new_features.py` print extra lines with
   `DEBUG_TRACK=1` – fine to keep, documented in the code.
5. Commits use this PC's git name/e-mail (`Hemanth-08-RA`); change with `git commit --amend --author=…`
   if they should be yours.
