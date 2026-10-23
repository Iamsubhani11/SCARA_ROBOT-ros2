# Changes

## 3.1.0 – 23 October 2026 – fits every phone, a real PC control room, sliders, twin test

**Fixed**
* **The app was cut off on most phones.** Everything narrower than 412 px (360 and 393 px are the
  common sizes) and any phone with large system text was wider than the screen: the header, tabs,
  readouts and the right joystick were sliced. The layout is now fluid from 320 px, survives 115 %
  system text (the app caps Android's text scaling there), and short phones get compact joysticks.
  A tab with more below fades at the bottom. Checked automatically at 320 / 360 / 393 / 430 px.
* **The browser opened on every start.** `scara_twin_server.py` and `launch_mujoco.py --gui` now
  only open it with `--open`; otherwise just go to http://localhost:8080.
* The arm-camera picture was drawn 10 px beside its frame on the PC page.
* In Twin mode the see-through real robot flickered through the simulated arm when both were in the
  same place. It now shows only where the robot is not where the simulation is.
* Colour buttons could overflow on 320 px phones with big text.

**New**
* **The PC page has its own design**: a dark control room with the tools on a rail, the 3-D cell in a
  machine-guard window, joints with limit bars (yellow near the end), the gripper position, and
  dialogs in the middle of the screen. The phone keeps the light pendant.
* **Sliders in the Manual tab**: one per joint and one for the gripper. Phone: a Joysticks / Sliders
  switch. PC: both at once.
* **Gesture** button right under the 3-D view (and in the Manual tab), with a clear message if the
  camera cannot start.
* **SUBHANI** in the header as a nameplate (like the letters on the shoulder link), on the start
  screen, in About and in the browser tab.
* **Direction check** (Robot tab): nudge each real joint and compare with the 3-D arm; *Opposite*
  reverses and saves it for every program.
* **The real robot's own limits apply in Twin / Real robot mode** (sliders, IK, moves), so the
  simulation never goes where the real arm cannot follow. Before, the firmware stopped the real arm
  at its limit while the simulation went on.
* Fixed: `config/twin_hardware.yaml` (joint signs / offsets) was only read by the ROS bridge – the
  app / website server, the webcam cell and the MuJoCo twin ignored it. All of them read it now.
* Fixed: the Gazebo vision node crashed without a screen (headless Gazebo, SSH, `--headless`) because
  it opened OpenCV windows - and without vision nothing was picked. Windows only open with a screen now.
* The project is arranged by topic: `mujoco/`, `gazebo/`, `real_robot/`, `server/`, `common/`
  (plus `mujoco/models`, `gazebo/worlds`, `real_robot/firmware`). The installed ROS layout is unchanged.
* Robot port errors say how to fix them (dialout group, port busy, pyserial missing).
* Dark and light mode on phone and PC (switch in the header, key T). The arm-camera window can be made
  smaller: pinch it on the phone, drag its corner with the mouse; double-click for full size again.
* Fixed: connected to MuJoCo, the app / website showed the arm sorting but **no pucks** in the
  Sorting and Command cells – the cell never sent their positions. It does now (belt, gripper, bins).
* **Motion graph** (FK / IK tab): the four joints over the last 10 s, and **Record → Save CSV**
  (time, joints, gripper position) for the graphs in a report. The website downloads the file; the
  phone app saves it on the PC (`~/scara_recordings/`).
* **Gripper path** in the 3-D view (FK / IK tab: show the gripper's path).
* **Robot watchdog**: a pulled USB cable or a hung Arduino is reported (the Real robot button blinks
  *NOT ANSWERING*) and nothing more is sent to it; when it answers again the twin carries on.
* The WebSocket server refuses oversized messages and unpacks big ones quickly.
* **No old bug comes back**: `docs/FIXED_BUGS.md` lists every bug fixed so far with the test that
  guards it; `bash tools/run_all_tests.sh` runs them all. New: `tools/test_static.py`,
  `tools/test_gazebo.py`, `tools/make_hand_video.py`, `tools/mujoco_harness.py` (the MuJoCo tests now
  run with `pip install mujoco` instead of a separate C program).
* `tools/test_twin_e2e.py`: the digital twin end to end without hardware – MuJoCo + the software
  robot: Twin moves, gripper, conveyor relay, STOP, world switches, and the app's IK Go moving both.

## 3.0.0 – 22 October 2026 – one app for phone and PC, FK / IK, voice, gesture control, hardware twin

**Bugs from the app videos**
* Pucks of the previous world stayed on the screen and overlapped the new ones. The Gazebo
  backend now forgets a puck it has not seen for 2 s (and any puck parked in the hidden pool),
  and the app clears the 3-D view the moment you pick another world.
* "The mode does not change after a while": the world buttons were greyed out for Gazebo, which
  had no worlds. The server now restarts Gazebo in the chosen world (`--sim gazebo --world ...`,
  about 30 s, with a "Starting the ... world" overlay); MuJoCo switches in a second.
* Messages piled up over the 3-D view. They now fade after 7 s; the full list is under
  Robot ▸ Activity.
* The arm camera swept through the forearm when the wrist turned. It is now fixed on the end of
  the forearm (it no longer turns with the wrist) and looks down at the gripper – the same in
  the URDF, Gazebo, MuJoCo, RViz, the GUI and the app.
* Readouts no longer show "-0.0°".

**New**
* **FK / IK tab** (app and GUI): live forward kinematics (X, Y, pad height, yaw, reach, elbow side,
  manipulability) in the robot or world frame, the formulas, and inverse kinematics – type a point
  or tap the table, see both elbow solutions with their joint limits, press Go. The area the
  gripper can really reach (joint limits included) is drawn on the table.
* **Voice** in the app: the phone's own recogniser (works offline when the phone has offline
  speech) or Chrome / Edge on a PC. "red then blue", "3 green", "any", "stop picking", "start",
  "stop", "open", "close", "home", "sorting", "command", "table", "emergency stop", "resume".
* **Gesture control** (button *Gesture* under the 3-D view, and in the Manual tab) with the front camera, no internet and no model files: move your hand to move
  the arm, make a fist to close the gripper. The camera preview sits in a corner of the 3-D view,
  away from the buttons.
* **Exact-position sliders** (Teach tab), **keyboard** in the GUI, an **Activity** log.
* **The GUI is now the app.** `scara_twin_server.py` serves the same page at
  http://localhost:8080 and connects it by itself; it also has the offline mode.
* **Hardware digital twin.** With `--sim hardware` the 3-D view shows the real cell as it stands
  on the table: the short red conveyor where you taught the pick point, the webcam on its stand,
  the drop spots, and every cube the webcam sees, at its real place on the belt
  (`tools/test_hw_twin.py`). The server also starts without a camera, so you can drive and
  teach the arm first.
* MIT licence in my name, `NOTICE.md` with the credits, a new README.

**Fixed while testing**
* The camera ruler crashed when its points were whole numbers in the JSON file.
* The "tap here" hint blocked taps on the 3-D view.
* Android: the camera-permission check was inverted.
* Slow phones moved the arm slower than asked (the joystick step was capped too low).

## October 2026 – Gazebo verified, real-robot camera cell, new phone app

**Gazebo**
* Ran the cell in Gazebo Harmonic for the first time end to end (headless, Jazzy builds of
  ros_gz and gz_ros2_control): belt, puck supply, depth camera, vision, controllers, sorting,
  command mode and manual control through the twin server.
* Fixed: puck poses had no names. `ros_gz_bridge` takes the frame name of a Pose_V entry from
  its header, but Gazebo puts model names in the pose's `name` field, so every transform on
  `/scara/world_poses` had an empty name. The result: new pucks were dropped onto pucks still
  waiting at the start of the belt (several pucks ended up inside each other), and sorted
  pucks were never cleared from the bins. New `scara_pose_relay.py` republishes the poses with
  their names; the launch file starts it instead of bridging that topic.
* Fixed: the first gripper command at start-up could be refused because the controller was
  loaded but not active yet – goals are now retried for a few seconds.
* `headless:=true` runs Gazebo without a window; the launch file finds `gz_ros2_control` in
  `~/gz_ros2_control_ws` by itself.
* The doctor no longer reports a stopped belt as a fault while the robot is picking.

**Real robot**
* Firmware pins, steps per unit, speeds and homing values now match the original
  HowToMechatronics sketch, value by value (J3 is 10 steps/°, homing order Z → J3 → J2 → J1 with the
  original step counts, parks at Z 100 mm). The 4 limit switches are now the default, like the
  original build, and the PC homes the robot with `K`. The conveyor relay sits on A1, a pin the
  original build leaves free. New `G <deg>` command to find the gripper's closed angle.
  Wiring diagram in `docs/`.
* New autonomous camera cell for the real robot with one 2-D webcam (Part 25 setup):
  `scara_camera_setup.py` (click-to-train colours, belt ruler), `scara_hw_cell.py` (teach poses,
  run sorting or command mode), also available in the GUI and the app
  (`scara_twin_server.py --sim hardware`). Dry-run tested against MuJoCo (`tools/test_hw_cell.py`).

**Phone app v2**
* Rebuilt from scratch: nothing overlaps any more, a target switch (Simulation / Real robot /
  Twin), safety and speed, teach & replay, sequences, tap-to-move a puck, robot zeroing and the
  camera-cell poses. Works with MuJoCo, Gazebo and the real robot through `scara_twin_server.py`.

**Phone app, second round**
* Picking with the joysticks now works offline too: the phone's own simulator (`app/ui/sim.js`) grabs a
  puck when the fingers are around it and close, drops it when they open, centres the gripper over a
  puck when it comes down near one, and stops a finger from pressing on a puck.
* New **Auto** tab: Table / Sorting / Command worlds – offline on the phone, or on the PC's MuJoCo,
  which now switches worlds when the app asks (`scara_twin_server.py --world`, `world` command).
  Start / Stop, sticky colours, sequences, a command box (type or dictate "red then blue"), totals.
* **XY + height** drive (straight lines over the table, relative to the 3-D view), tilt-to-drive,
  gripper slider, Follow view, reach ring, vibration feedback, offline teach & replay, add puck / reset,
  picks per minute, and a how-to guide behind the **?** button.
* Server: `move` (absolute joint target) and `world` commands; the scene sent to the app carries the
  kinematics and cell geometry.

**Other**
* Sequence mode: `sequence red blue green` (or "red then blue then green").
* Speed slider for every manual control (GUI and app).
* Tap-to-move: pick a puck on the table and put it somewhere else (`pick_place` command).
* Name on the shoulder link: the raised letters are merged into `shoulder_link.stl` itself (deleting a
  file can't remove the name); a thin second mesh paints them white (`tools/make_name_mesh.py`,
  `--remove` takes them off, `tools/blender_name.py`).
* README rewritten.
* The old v1.1 app and its patch file are gone - v2 replaces them.
