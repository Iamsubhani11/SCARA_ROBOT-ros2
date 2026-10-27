# Fixed bugs – and the test that keeps each one fixed

Every bug found in this project so far, what caused it, and which automatic check fails if it ever
comes back. Run them all with:

```bash
bash tools/run_all_tests.sh          # everything this computer can run; prints a summary
```

Rule for every change: **run `tools/run_all_tests.sh` before you commit.** A new bug gets a line here and
a check in one of the tests.

| # | Bug (what you saw) | Cause | Guarded by |
|---|---|---|---|
| 1 | Gazebo: new pucks dropped into pucks still waiting; sorted pucks never cleared | `ros_gz_bridge` gave every puck pose an empty name | `gazebo/scara_pose_relay.py` · `tools/test_gazebo.py` (picking) |
| 2 | Gazebo: first gripper command refused at start-up | controller loaded but not active yet | sorter retries · `tools/test_gazebo.py` |
| 3 | Real robot: wrong pins / steps / homing compared with the original build | firmware numbers differed from the HowToMechatronics sketch | `tools/test_firmware_host.py` |
| 4 | App: objects of the previous world stayed and overlapped the new ones | Gazebo backend kept stale pucks; app did not clear on switch | `test_app_v3.py` (worlds), `test_twin_e2e.py`, `test_gazebo.py` |
| 5 | App: after a while the world (mode) did not change any more | world buttons disabled for Gazebo – it had no worlds | `test_gazebo.py` (world switches), `test_twin_e2e.py` |
| 6 | Arm camera went inside the shoulder link when the wrist turned | camera hung on the wrist | `test_static.py` (URDF + 3 MuJoCo scenes: camera on the forearm) |
| 7 | Messages piled up over the 3-D view | ticker never expired | `test_app_v3.py` (old messages leave) |
| 8 | Readouts showed "-0.0°" | rounding of tiny negatives | `test_app_v3.py` (readouts) |
| 9 | Voice: "let go" started the belt, "release" was ambiguous | order of the phrase rules | `test_static.py` (voice phrases), `test_app_v3.py` |
| 10 | Webcam cell crashed when the ruler points were whole numbers | integer numpy array divided in place | `test_hw_twin.py` (uses whole-number ruler points) |
| 11 | "Tap the table" / "Tap to move a puck" did nothing on small screens | the yellow hint covered the 3-D view and ate the taps | `test_app_v3.py` (tap the table) |
| 12 | Android: hand control asked for the camera although it was allowed (and the other way round) | permission check inverted in `Bridge.smali` | `test_static.py` (Android hasCamera) |
| 13 | Joysticks moved slower than asked on slow phones | per-frame step capped at 0.1 s | `test_app_v3.py` (keyboard drive distance) |
| 14 | **Phone app cut off on the right** (header, tabs, readouts, right joystick) on phones narrower than 412 px or with big system text | grid column grew to its content; nothing could shrink | `test_app_v3.py` (320 / 360 / 393 / 430 px, 100 % and 115 % text, every tab) |
| 15 | Joint values cut ("-110 …") in the readout boxes | number font too big for 5 boxes | `test_app_v3.py` (longest values fit) |
| 16 | Title cut ("SCARA Digital Tw…") on 412 px phones | header too wide after the dark/light button | `test_app_v3.py` (title not cut) |
| 17 | Browser GUI opened by itself on every start | `serve()` opened it by default | `test_static.py` (only with `--open`) |
| 18 | Arm-camera picture drawn 10 px beside its white frame on the PC | renderer used a fixed 8 px offset | `test_app_v3.py` (picture aligned, phone and PC) |
| 19 | Twin: the see-through real robot flickered through the sim arm | two arms drawn in the same place | `test_twin_e2e.py` (no second arm when in sync) |
| 20 | Colour buttons overflowed on 320 px phones with big text | grid columns could not shrink | `test_app_v3.py` (widths, Auto tab) |
| 21 | **Gazebo picked nothing without a screen** (headless, SSH, `--headless`) | vision node opened OpenCV windows; Qt aborted the node | `gazebo/scara_vision_node.py` checks for a screen · `test_gazebo.py` |
| 22 | Joint direction fixes in `config/twin_hardware.yaml` had no effect in the app / website | only the ROS bridge read that file | `test_static.py` (settings read, saved change used) |
| 23 | Twin: simulation went past the real robot's range while the robot stopped at its limit | only URDF limits were applied | `test_twin_e2e.py` (both stop at −89°) |
| 24 | "Connect" without a port silently connected a *software* robot | `open_link(None)` meant fake | `test_twin_e2e.py` (refused) |
| 25 | Pulled USB cable / hung Arduino: app still said "connected", twin carried on | no watchdog | `test_twin_e2e.py` (robot silent → NOT ANSWERING → recovers) |
| 26 | A huge or broken WebSocket message could eat the PC's memory | no size limit, slow byte-by-byte unmasking | `test_static.py` (websocket) |
| 27 | Sliders showed 0° before homing while the robot was elsewhere | showed stale targets | app shows the robot's pose until homed |
| 28 | Programs still looked for the old `scripts/` folder after the reorganisation | paths | `test_static.py` (no `scripts/`), CMake/launch checks |
| 29 | **App / website connected to MuJoCo: the arm sorted, but no pucks were drawn** in the Sorting and Command cells (the Table showed them) | the MuJoCo conveyor-cell backend never reported puck positions | `test_twin_e2e.py` (pucks reach the app in Sorting and Command, and the phone app draws them) |
| 30 | Keyboard keys (WASD) did nothing while a slider had the focus; a keyboard plugged into the phone did nothing until the page was tapped | key handler ignored every input field; the WebView did not take focus | `test_camera_gamepad.py` (keys with the speed slider focused), Android shell requests focus |
| 31 | Gesture camera covered or shrank the robot view | camera picture drawn inside the 3-D view | `test_camera_gamepad.py` (Docked / Window / Small never cover or shrink the robot view; phone, tablet, PC) |
| 32 | **Phone app 3.2.0: joysticks seemed reversed, the wrist kept turning and the height drifted by itself** | a device the phone reports as a gamepad has triggers resting at -1; they were read as a right stick held over | `test_camera_gamepad.py` (phantom pad and Xbox-style triggers must not move the arm); a pad now only works after a person uses it |

Lesson from #29: a test must *expect* something, not just print it – the old test printed "0 pucks" and
still passed. Every check here now fails when the expected thing is missing.
