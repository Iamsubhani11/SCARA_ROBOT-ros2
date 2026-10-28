# Changes

## 3.3.0 – 27 October 2026 – real hand tracking, new modes, hardware bring-up

**New**
* **Gesture control with MediaPipe** in the app and the website. MediaPipe Hand Landmarker finds 21
  points on each hand; everything it needs (JavaScript, WebAssembly, the 7.8 MB model) is inside
  `app/ui/lib/mediapipe`, so it works offline in the browser and in the Android app. The palm is the
  left joystick (same still zone as before), a fist closes the gripper and an open hand opens it,
  a **pinch** turns the hand into the right joystick (pinch and move up / down = height in XY drive),
  and with **two hands** the second one is the right joystick. A hand only drives after it has been
  in its circle once, so a new hand never makes the arm jump. The skeleton is drawn on the camera
  picture and the window says which tracker runs. The old colour tracker is kept: it takes over by
  itself if MediaPipe cannot start, and the ✋ button switches between them.
* **The camera window says why** when gesture control cannot start: permission refused (browser or
  Android), the page opened over plain http from another computer, no camera, camera busy, too dark
  or too bright.
* **Draw a path** (FK / IK tab): draw on the table with a finger or the mouse, or choose a circle,
  square, star or figure 8. *Run path* moves the gripper along it with the inverse kinematics
  (red parts are out of reach). Works offline, with MuJoCo, Gazebo and the real robot.
* **Programs** (Teach tab): save the poses as a named program, load it, **export** it as a `.json`
  file and **import** it again (the phone app saves the file on the PC in `~/scara_programs/`, or
  shows it as text to copy when offline).
* **Belt tracking** (Auto tab → *Follow belt*, command cell): the robot picks a puck from the
  moving belt instead of stopping it. The puck's path is known from the camera and the belt speed;
  the arm meets it and follows it down the belt with the Jacobian (resolved-rate: q̇ = J⁻¹·v) while it
  goes down, closes and lifts. The belt speed is measured by the camera. Offline and in MuJoCo
  (`launch_mujoco.py --command --track-belt`). The belt only pauses while the arm is away at the bin.
  **Switched off in Gazebo for now**: Gazebo's arm controller lags its trajectory by an amount that
  changes from pick to pick; with a fixed 0.2 s lead 1 of 6 tracked picks held. Gazebo keeps stopping
  the belt (README section 9).
  It is for the command cell: in the sorting cell, where every puck is wanted and they come every
  2.5 s, our tests showed it loses pucks, so sorting keeps stopping the belt.
* **Demo for the review** (Auto tab): the table (the gripper draws a star, then moves a puck), the
  sorting cell and the command cell ("red then blue", then belt tracking), one after another.
* **Hardware bring-up check** `real_robot/scara_bringup.py`: the first start of the real robot step
  by step – connection, homing, every joint's direction (saved for every program), gripper, relay,
  and a digital-twin check. `--fake --yes` runs it with the software robot.
* **docs/HARDWARE_GUIDE.md**: building and connecting the robot in one page, for both builds
  (with limit switches, relay on A1 – or Mechamind-style without switches, relay on D10).

**Fixed** (details and tests in docs/FIXED_BUGS.md, #33–#38)
* Gazebo picked nothing and showed no pucks after `colcon build --symlink-install` – the programs
  could not find the shared code through the symlinks (it looked like a Gazebo crash).
* Gazebo's controllers sometimes did not start (slow first start, or an old Gazebo still running).
  The spawners now wait 60 s and an old SCARA Gazebo is stopped first.
* XY drive: the gripper turned while moving and the arm kept going after the stick was let go.
* Tilt drive did nothing in new Chrome / Android WebView.
* The digital-twin test failed on PCs where MuJoCo was installed with `pip install --user`.
* The kinematics self-test only printed its numbers; it now checks them (and the Jacobian).
* Offline simulator: a finger already beside a puck stopped the gripper from closing on it when the
  puck was a few mm off centre (a real parallel gripper pushes it to the middle).

**Checked automatically** – new tests: `tools/test_gesture_mediapipe.py` (a video made from real hand
photos; MediaPipe offline from `file://` like the Android app and over http like the website),
`tools/test_new_features.py` (paths, programs, belt tracking, demo), `MuJoCo belt tracking`
(`sim_test_sorting.py` with `TRACK=1`), `hardware bring-up (software robot)`.

**Test run for this release** (28 October 2026, Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic 8.15,
MuJoCo 3.12, Chromium 156): `bash tools/run_all_tests.sh` – **21 passed, 0 failed, 0 skipped**, Gazebo
included. (One check of the digital-twin test, "the table world shows its 6 pucks", read the state too
early after a world switch and failed once; it now waits up to 10 s and prints the count.) Before the
3.3 work the same suite gave 13 passed, 4 failed (bugs #33–#36).

## 3.2.1 – 26 October 2026 – joysticks fixed on phones

**Fixed**
* **On some phones the joysticks seemed reversed and the arm kept moving by itself** (the wrist kept
  turning, the height drifted). The phone reported a device that looks like a gamepad; its triggers
  rest at -1, and 3.2.0 read them as a right stick held over. Now a gamepad does nothing until a person
  actually uses it (a button press or a real stick push), every axis is measured from where it rests,
  and an axis that does not rest at 0 (a trigger) is never used as a stick. The on-screen joysticks are
  exactly as in 3.1.0. Guarded by `tools/test_camera_gamepad.py` (a phantom pad must not move the arm,
  an Xbox-style pad's triggers must not turn the wrist).

## 3.2.0 – 25 October 2026 – gesture camera window, gamepads, keyboards on the phone

**New**
* **Gesture camera in its own place**, like the camera window of Gazebo / MuJoCo. It never covers
  or shrinks the robot view: on a phone it sits above the 3-D view (the page scrolls to the
  controls), on a tablet or PC at the top of the right-hand column. Drag the bar under the picture
  to make it taller or shorter. **Window** makes it a floating window (drag the title, resize from
  the corner), **Small** a corner preview (tap to dock it again), ✕ closes it and stops gesture
  control. Mode, size and place are remembered. The picture now shows what the tracker sees: the
  learn square with a countdown, the still zone, the full-speed ring, your hand and open / fist.
  Double-tap the picture to learn your hand again.
* **External gamepads** (USB or Bluetooth – Zebronics, Xbox, PlayStation, generic) on the website and
  in the Android app: sticks and d-pad drive, A close, B open, X home, Y save pose, LB / RB speed,
  Start STOP, Select belt. Pads without the standard layout (right stick on Z / Rz, hat switch) work
  too.
* **Keyboards on the phone** (USB-OTG or Bluetooth) work like on the PC. Arrow keys drive the left
  stick, + / - change the speed, X switches joints / XY. The keys (and the gamepad's buttons) are
  shown in the Manual tab as soon as one is used.

**Fixed**
* WASD did nothing while a slider (speed, gripper) had the focus.
* The Android app now gives the page the keyboard focus at start, so a plugged-in keyboard works
  without tapping the page first.
* Checked automatically: `tools/test_camera_gamepad.py` (every camera mode at phone, tablet and PC
  sizes, a gamepad driving the arm and the gripper, keyboard keys).

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
