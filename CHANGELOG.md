# Changes

## 3.0.0 – 22 October 2026 – one app for phone and PC, FK / IK, voice, hand control, hardware twin

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
* **Hand control** with the front camera, no internet and no model files: move your hand to move
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
  `--remove` for the plain arm, `tools/blender_name.py`).
* README rewritten.
* The old v1.1 app and its patch file are gone - v2 replaces them.
